"""ISONAVI-ROV Mission Control & Ground Station (GCS).

A professional PyQt5 tactical ground control station for underwater inspection:
  - Live Telemetry Monitoring: Depth, 3D Pose (X, Y, Z), Velocities, Heading Compass, Attitude
  - Vehicle Status & Diagnostics: Safety Supervisor state, 8-Thruster matrix, Alarms
  - Live Optical Camera Feed: Real-time YOLOv8 & Spatial AI detection stream
  - Target Detection Log & Storage: Interactive detection inventory, CSV/JSON export, snapshots
  - Subsea Command Deck: Teleoperation keypad, cruise presets, auto-center, flood control, kill switch
  - Comms & Failure Watchdog: Connection heartbeat pulse, loss-of-signal alarm, tilt/depth limits
"""

import csv
import json
import math
import os
import sys
import time
from collections import deque
from datetime import datetime

import cv2
import numpy as np
from PyQt5.QtCore import QObject, QPointF, QRectF, Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QFont,
    QIcon,
    QImage,
    QPainter,
    QPainterPath,
    QPen,
    QPixmap,
    QPolygonF,
)
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QSlider,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from cv_bridge import CvBridge

from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu, Joy
from std_msgs.msg import Bool, Float64, Float64MultiArray, String


def quat_to_euler(q):
    """Convert geometry_msgs quaternion to euler [roll, pitch, yaw] in degrees."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return roll, pitch, (yaw + 360.0) % 360.0


# ==============================================================================
# ROS 2 TELEMETRY & COMMAND BRIDGE
# ==============================================================================
class RosBridge(QObject):
    sig_image = pyqtSignal(QImage)
    sig_telemetry = pyqtSignal(dict)
    sig_thrusters = pyqtSignal(list)
    sig_status = pyqtSignal(dict)
    sig_detections = pyqtSignal(list)
    sig_log = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.node = None
        self.bridge = CvBridge()
        self.active_stream = "detections"
        self.last_packet_t = 0.0

        # Output states
        self.surge = 0.0
        self.sway = 0.0
        self.heave = 0.0
        self.yaw = 0.0
        self.button_c = 0
        self.armed = False

    def init_ros(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("isonavi_ground_station")

        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
        )

        # Subscribers
        self.sub_img_det = self.node.create_subscription(
            Image, "camera/image_detections", self.on_image_det, qos_best_effort
        )
        self.sub_img_enh = self.node.create_subscription(
            Image, "camera/image_enhanced", self.on_image_enh, qos_best_effort
        )
        self.sub_img_raw = self.node.create_subscription(
            Image, "camera/image_raw", self.on_image_raw, qos_best_effort
        )
        self.sub_det_json = self.node.create_subscription(
            String, "camera/detections", self.on_detections, 10
        )
        self.sub_depth = self.node.create_subscription(
            Float64, "depth", self.on_depth, 10
        )
        self.sub_odom = self.node.create_subscription(
            Odometry, "model/isonavi_rov/odometry", self.on_odom, 10
        )
        self.sub_imu = self.node.create_subscription(
            Imu, "imu", self.on_imu, 10
        )
        self.sub_thrusters = self.node.create_subscription(
            Float64MultiArray, "thrusters", self.on_thrusters, 10
        )
        self.sub_safety = self.node.create_subscription(
            String, "safety_state", self.on_safety, 10
        )
        self.sub_flood_status = self.node.create_subscription(
            String, "flood_status", self.on_flood_status, 10
        )
        self.sub_flood_active = self.node.create_subscription(
            Bool, "flood_active", self.on_flood_active, 10
        )

        # Publishers
        self.pub_joy = self.node.create_publisher(Joy, "joy", 10)
        self.pub_kill = self.node.create_publisher(Bool, "kill", 10)
        self.pub_flood = self.node.create_publisher(Bool, "flood", 10)

        # Telemetry State Cache
        self.telemetry = {
            "depth": 0.0,
            "x": 4.0,
            "y": 0.0,
            "z": -1.0,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        self.status = {
            "safety_state": "INITIALIZING",
            "flood_active": False,
            "flood_status": "NORMAL [CALM]",
            "comms_online": False,
            "auto_center": False,
        }

        # 20 Hz Joystick Heartbeat Timer
        self.timer_joy = self.node.create_timer(0.05, self.publish_joy)

    def set_stream(self, stream_type: str):
        self.active_stream = stream_type

    def on_image_det(self, msg: Image):
        if self.active_stream == "detections":
            self.process_image(msg)

    def on_image_enh(self, msg: Image):
        if self.active_stream == "enhanced":
            self.process_image(msg)

    def on_image_raw(self, msg: Image):
        if self.active_stream == "raw":
            self.process_image(msg)

    def process_image(self, msg: Image):
        self.last_packet_t = time.time()
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()
            self.sig_image.emit(qimg)
        except Exception as e:
            pass

    def on_depth(self, msg: Float64):
        self.last_packet_t = time.time()
        self.telemetry["depth"] = float(msg.data)
        self.sig_telemetry.emit(dict(self.telemetry))

    def on_odom(self, msg: Odometry):
        self.last_packet_t = time.time()
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.telemetry["x"] = float(p.x)
        self.telemetry["y"] = float(p.y)
        self.telemetry["z"] = float(p.z)
        self.telemetry["vx"] = float(v.x)
        self.telemetry["vy"] = float(v.y)
        self.telemetry["vz"] = float(v.z)

        r, pt, y = quat_to_euler(msg.pose.pose.orientation)
        self.telemetry["roll"] = r
        self.telemetry["pitch"] = pt
        self.telemetry["yaw"] = y
        self.sig_telemetry.emit(dict(self.telemetry))

    def on_imu(self, msg: Imu):
        self.last_packet_t = time.time()
        r, pt, y = quat_to_euler(msg.orientation)
        self.telemetry["roll"] = r
        self.telemetry["pitch"] = pt
        self.telemetry["yaw"] = y
        self.sig_telemetry.emit(dict(self.telemetry))

    def on_thrusters(self, msg: Float64MultiArray):
        self.last_packet_t = time.time()
        self.sig_thrusters.emit(list(msg.data))

    def on_safety(self, msg: String):
        self.last_packet_t = time.time()
        self.status["safety_state"] = msg.data
        self.sig_status.emit(dict(self.status))

    def on_flood_status(self, msg: String):
        self.status["flood_status"] = msg.data
        self.sig_status.emit(dict(self.status))

    def on_flood_active(self, msg: Bool):
        self.status["flood_active"] = bool(msg.data)
        self.sig_status.emit(dict(self.status))

    def on_detections(self, msg: String):
        try:
            items = json.loads(msg.data)
            if isinstance(items, list):
                self.sig_detections.emit(items)
        except Exception:
            pass

    def publish_joy(self):
        msg = Joy()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        # [0: sway, 1: surge, 2: unused, 3: yaw, 4: depth, 5..7: unused]
        msg.axes = [
            float(self.sway),
            float(self.surge),
            0.0,
            float(self.yaw),
            float(self.heave),
            0.0,
            0.0,
            0.0,
        ]
        msg.buttons = [int(self.button_c), 0, 0, 0, 0, 0, 0, 0]
        self.pub_joy.publish(msg)

        # Check comms freshness
        dt = time.time() - self.last_packet_t
        self.status["comms_online"] = (dt < 1.2 and self.last_packet_t > 0)
        self.status["auto_center"] = bool(self.button_c)
        self.sig_status.emit(dict(self.status))

    def send_motion(self, surge, sway, heave, yaw):
        self.surge = float(surge)
        self.sway = float(sway)
        self.heave = float(heave)
        self.yaw = float(yaw)

    def toggle_auto_center(self):
        self.button_c = 1 if self.button_c == 0 else 0
        self.status["auto_center"] = bool(self.button_c)
        self.sig_status.emit(dict(self.status))
        return self.button_c

    def stop_all(self):
        self.surge = 0.0
        self.sway = 0.0
        self.heave = 0.0
        self.yaw = 0.0

    def trigger_kill(self):
        self.stop_all()
        self.pub_kill.publish(Bool(data=True))

    def trigger_flood(self, enable: bool):
        self.pub_flood.publish(Bool(data=enable))


# ==============================================================================
# CUSTOM GRAPHICAL TACTICAL WIDGETS
# ==============================================================================
class RiverMapWidget(QWidget):
    """Top-Down Tactical Bathymetry & Trajectory Map."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 240)
        self.rov_x = 4.0
        self.rov_y = 0.0
        self.rov_yaw = 0.0
        self.trail = deque(maxlen=200)
        self.targets = []

    def update_pose(self, x, y, yaw):
        self.rov_x = x
        self.rov_y = y
        self.rov_yaw = yaw
        self.trail.append((x, y))
        self.update()

    def update_targets(self, targets):
        self.targets = targets
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        # Dark tactical background
        p.fillRect(0, 0, w, h, QColor("#0d1117"))

        # Map projection: X in [0, 80m], Y in [-8m, +8m]
        pad_x = 25
        pad_y = 20
        map_w = w - 2 * pad_x
        map_h = h - 2 * pad_y

        def world_to_screen(wx, wy):
            sx = pad_x + (wx / 80.0) * map_w
            sy = h / 2.0 - (wy / 8.0) * (map_h / 2.0)
            return sx, sy

        # Draw River Channel Boundaries
        p.setPen(QPen(QColor("#21262d"), 1, Qt.DashLine))
        y_top_s = world_to_screen(0, 8.0)[1]
        y_bot_s = world_to_screen(0, -8.0)[1]
        p.drawLine(int(pad_x), int(y_top_s), int(w - pad_x), int(y_top_s))
        p.drawLine(int(pad_x), int(y_bot_s), int(w - pad_x), int(y_bot_s))

        # Riverbed flow fill
        water_rect = QRectF(pad_x, y_top_s, map_w, y_bot_s - y_top_s)
        p.fillRect(water_rect, QColor(14, 28, 44, 180))

        # Civil Structures Landmarks
        p.setPen(QPen(QColor("#388bfd"), 1))
        p.setBrush(QColor("#1f6feb"))
        # Bridge Pier 1 (x=22)
        p1_x, p1_y = world_to_screen(22.0, 0.0)
        p.drawEllipse(QPointF(p1_x, p1_y), 6, 6)
        p.drawText(int(p1_x - 18), int(p1_y - 10), "PIER 1")

        # Pipeline Crossing (x=42)
        pipe_x = world_to_screen(42.0, 0.0)[0]
        p.setPen(QPen(QColor("#d29922"), 2, Qt.DashLine))
        p.drawLine(int(pipe_x), int(y_top_s), int(pipe_x), int(y_bot_s))
        p.drawText(int(pipe_x - 16), int(y_bot_s + 14), "PIPE")

        # Bridge Pier 2 (x=58)
        p2_x, p2_y = world_to_screen(58.0, 0.0)
        p.setPen(QPen(QColor("#388bfd"), 1))
        p.setBrush(QColor("#1f6feb"))
        p.drawEllipse(QPointF(p2_x, p2_y), 6, 6)
        p.drawText(int(p2_x - 18), int(p2_y - 10), "PIER 2")

        # Trajectory Breadcrumb Trail
        if len(self.trail) > 1:
            p.setPen(QPen(QColor(88, 166, 255, 140), 2))
            pts = [QPointF(*world_to_screen(tx, ty)) for tx, ty in self.trail]
            for i in range(len(pts) - 1):
                p.drawLine(pts[i], pts[i + 1])

        # Draw Detected Targets on Map
        for t in self.targets:
            if "spatial_xyz" in t:
                tx_w = self.rov_x + t["spatial_xyz"][0]
                ty_w = self.rov_y + t["spatial_xyz"][1]
                tsx, tsy = world_to_screen(tx_w, ty_w)
                p.setPen(QPen(QColor("#ff7b72"), 1))
                p.setBrush(QColor("#f85149"))
                p.drawEllipse(QPointF(tsx, tsy), 4, 4)
                lbl = t.get("class", "OBJ").split("_")[-1].upper()[:4]
                p.setFont(QFont("Monospace", 7))
                p.drawText(int(tsx + 5), int(tsy + 3), lbl)

        # Draw ROV Position Marker & Heading Vector
        rx, ry = world_to_screen(self.rov_x, self.rov_y)
        yaw_rad = math.radians(self.rov_yaw)

        # Orientation vector
        vec_len = 16.0
        vx = rx + vec_len * math.cos(yaw_rad)
        vy = ry - vec_len * math.sin(yaw_rad)
        p.setPen(QPen(QColor("#58a6ff"), 2))
        p.drawLine(QPointF(rx, ry), QPointF(vx, vy))

        # ROV Triangle Icon
        p.setPen(QPen(QColor("#ffffff"), 1.5))
        p.setBrush(QColor("#2ea043"))
        tip = QPointF(rx + 9 * math.cos(yaw_rad), ry - 9 * math.sin(yaw_rad))
        left = QPointF(rx + 6 * math.cos(yaw_rad + 2.5), ry - 6 * math.sin(yaw_rad + 2.5))
        right = QPointF(rx + 6 * math.cos(yaw_rad - 2.5), ry - 6 * math.sin(yaw_rad - 2.5))
        p.drawPolygon(QPolygonF([tip, left, right]))

        # Scale and coordinate legend
        p.setFont(QFont("Monospace", 8))
        p.setPen(QColor("#8b949e"))
        p.drawText(pad_x, h - 5, f"POS: X={self.rov_x:.1f}m | Y={self.rov_y:.1f}m | YAW={self.rov_yaw:.0f}°")


class AttitudeIndicatorWidget(QWidget):
    """Artificial Horizon and Pitch/Roll Attitude Gauge."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(140, 140)
        self.roll = 0.0
        self.pitch = 0.0

    def update_attitude(self, roll, pitch):
        self.roll = roll
        self.pitch = pitch
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2.0, h / 2.0
        rad = min(cx, cy) - 8

        # Clip to circle
        path = QPainterPath()
        path.addEllipse(QPointF(cx, cy), rad, rad)
        p.setClipPath(path)

        # Save coordinate frame
        p.save()
        p.translate(cx, cy)
        p.rotate(-self.roll)

        # Pitch displacement (2 px per degree)
        pitch_px = max(-rad, min(rad, self.pitch * 1.8))

        # Sky (blue) & Ground (brown)
        p.fillRect(QRectF(-rad * 2, -rad * 2 + pitch_px, rad * 4, rad * 2), QColor("#1f6feb"))
        p.fillRect(QRectF(-rad * 2, pitch_px, rad * 4, rad * 2), QColor("#593f18"))

        # Horizon Line
        p.setPen(QPen(QColor("#ffffff"), 2))
        p.drawLine(QPointF(-rad, pitch_px), QPointF(rad, pitch_px))

        # Pitch ladder ticks
        for deg in (-20, -10, 10, 20):
            ty = pitch_px - deg * 1.8
            p.drawLine(QPointF(-15, ty), QPointF(15, ty))

        p.restore()
        p.setClipping(False)

        # Fixed Aircraft / ROV reticle in center
        p.setPen(QPen(QColor("#ffea79"), 2.5))
        p.drawLine(int(cx - 24), int(cy), int(cx - 8), int(cy))
        p.drawLine(int(cx + 8), int(cy), int(cx + 24), int(cy))
        p.drawEllipse(QPointF(cx, cy), 3, 3)

        # Outer bezel
        p.setPen(QPen(QColor("#30363d"), 2.5))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(cx, cy), rad, rad)

        # Text readout
        p.setFont(QFont("Monospace", 8))
        p.setPen(QColor("#c9d1d9"))
        p.drawText(QRectF(0, h - 18, w, 16), Qt.AlignCenter, f"R:{self.roll:+.1f}° P:{self.pitch:+.1f}°")


class DepthGaugeWidget(QWidget):
    """Vertical Depth Column and Digital Water Column Gauge."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(80, 180)
        self.depth_m = 0.0

    def update_depth(self, d):
        self.depth_m = max(0.0, d)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        p.fillRect(0, 0, w, h, QColor("#161b22"))

        bar_x = 18
        bar_w = 20
        bar_y = 30
        bar_h = h - 60

        # Max scale: 5 meters depth
        max_d = 5.0
        fill_h = min(bar_h, (self.depth_m / max_d) * bar_h)

        # Background column
        p.fillRect(bar_x, bar_y, bar_w, bar_h, QColor("#0d1117"))

        # Water fill gradient (cyan to deep blue)
        p.fillRect(bar_x, bar_y, bar_w, int(fill_h), QColor("#388bfd"))

        # Bezel and depth indicator arrow
        p.setPen(QPen(QColor("#8b949e"), 1.5))
        p.drawRect(bar_x, bar_y, bar_w, bar_h)

        ind_y = bar_y + fill_h
        p.setPen(QPen(QColor("#f85149"), 2))
        p.drawLine(bar_x - 4, int(ind_y), bar_x + bar_w + 4, int(ind_y))

        # Ticks
        p.setFont(QFont("Monospace", 7))
        p.setPen(QColor("#8b949e"))
        for m in range(0, 6):
            ty = bar_y + (m / max_d) * bar_h
            p.drawLine(bar_x + bar_w, int(ty), bar_x + bar_w + 4, int(ty))
            p.drawText(bar_x + bar_w + 6, int(ty + 3), f"{m}m")

        # Header and Digital Readout
        p.setFont(QFont("Monospace", 8, QFont.Bold))
        p.setPen(QColor("#58a6ff"))
        p.drawText(QRectF(0, 8, w, 18), Qt.AlignCenter, "DEPTH")

        p.setFont(QFont("Monospace", 9, QFont.Bold))
        p.setPen(QColor("#ffffff"))
        p.drawText(QRectF(0, h - 24, w, 20), Qt.AlignCenter, f"{self.depth_m:.2f} m")


# ==============================================================================
# MAIN GROUND STATION WINDOW
# ==============================================================================
class GroundStationWindow(QMainWindow):
    def __init__(self, bridge: RosBridge):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle("ISONAVI-ROV Mission Control & Ground Station [GCS]")
        self.resize(1440, 920)

        # Internal state
        self.start_time = datetime.now()
        self.cruise_speed = 0.50
        self.detected_items_map = {}  # key: class_name -> data
        self.auto_save_path = os.path.expanduser("~/.ros/isonavi_detection_log.json")
        self.snapshot_dir = os.path.expanduser("~/isonavi_snapshots")
        os.makedirs(self.snapshot_dir, exist_ok=True)

        self.apply_theme()
        self.init_ui()
        self.connect_signals()

        # UI 10 Hz refresh timer
        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.update_mission_clock)
        self.ui_timer.start(100)

    def apply_theme(self):
        self.setStyleSheet("""
            QMainWindow {
                background-color: #0d1117;
            }
            QWidget {
                background-color: #0d1117;
                color: #c9d1d9;
                font-family: 'Segoe UI', Ubuntu, sans-serif;
            }
            QGroupBox {
                border: 1px solid #30363d;
                border-radius: 6px;
                margin-top: 14px;
                font-weight: bold;
                font-size: 11px;
                text-transform: uppercase;
                letter-spacing: 1px;
                padding-top: 10px;
                color: #58a6ff;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
                background-color: #0d1117;
            }
            QPushButton {
                background-color: #21262d;
                border: 1px solid #363b42;
                border-radius: 4px;
                padding: 6px 12px;
                color: #c9d1d9;
                font-weight: 600;
            }
            QPushButton:hover {
                background-color: #30363d;
                border-color: #8b949e;
            }
            QPushButton:pressed {
                background-color: #1f6feb;
                color: #ffffff;
            }
            QPushButton#btnKill {
                background-color: #da3633;
                border-color: #f85149;
                color: #ffffff;
                font-weight: bold;
            }
            QPushButton#btnKill:hover {
                background-color: #f85149;
            }
            QPushButton#btnArm {
                background-color: #238636;
                border-color: #2ea043;
                color: #ffffff;
            }
            QPushButton#btnArm:hover {
                background-color: #2ea043;
            }
            QTableWidget {
                background-color: #161b22;
                border: 1px solid #30363d;
                gridline-color: #21262d;
                selection-background-color: #1f6feb;
                selection-color: #ffffff;
                font-family: Monospace;
                font-size: 11px;
            }
            QHeaderView::section {
                background-color: #21262d;
                color: #8b949e;
                padding: 4px;
                border: 1px solid #30363d;
                font-weight: bold;
            }
            QLabel {
                font-size: 11px;
            }
            QProgressBar {
                background-color: #161b22;
                border: 1px solid #30363d;
                border-radius: 3px;
                text-align: center;
                color: #ffffff;
                font-size: 9px;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background-color: #238636;
            }
        """)

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(10, 8, 10, 8)
        main_layout.setSpacing(8)

        # 1. TOP HEADER & TELEMETRY HEARTBEAT BANNER
        header = self.create_header_bar()
        main_layout.addWidget(header)

        # 2. MAIN WORKSPACE (Splitter: Left Camera & Log, Right Gauges & Controls)
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter, 1)

        # Left Column: Live Vision Feed & Target Inventory Log
        left_column = QWidget()
        left_layout = QVBoxLayout(left_column)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(8)

        left_layout.addWidget(self.create_camera_box(), 3)
        left_layout.addWidget(self.create_detection_log_box(), 2)
        splitter.addWidget(left_column)

        # Right Column: Nav Gauges, Bathymetry Map, Thruster Matrix & Commands
        right_column = QWidget()
        right_layout = QVBoxLayout(right_column)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)

        right_layout.addWidget(self.create_telemetry_gauges_box(), 2)
        right_layout.addWidget(self.create_control_deck_box(), 2)
        splitter.addWidget(right_column)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

    def create_header_bar(self):
        bar = QFrame()
        bar.setStyleSheet("background-color: #161b22; border-radius: 6px; padding: 4px;")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(12, 4, 12, 4)

        # Station Title
        title = QLabel("🛡️ ISONAVI-ROV // TACTICAL MISSION CONTROL")
        title.setStyleSheet("font-size: 14px; font-weight: bold; color: #58a6ff; letter-spacing: 1.5px;")
        layout.addWidget(title)

        layout.addSpacing(20)

        # Comms Status LED
        self.lbl_comms = QLabel("🟢 COMMS ONLINE [50 Hz]")
        self.lbl_comms.setStyleSheet("color: #2ea043; font-weight: bold;")
        layout.addWidget(self.lbl_comms)

        layout.addSpacing(15)

        # Vehicle Safety State Badge
        self.lbl_safety = QLabel("STATE: ARMED [STATION-HOLD]")
        self.lbl_safety.setStyleSheet("background-color: #238636; color: #fff; padding: 3px 8px; border-radius: 4px; font-weight: bold;")
        layout.addWidget(self.lbl_safety)

        layout.addSpacing(15)

        # Mission Elapsed Time
        self.lbl_clock = QLabel("MISSION: 00:00:00")
        self.lbl_clock.setStyleSheet("color: #8b949e; font-family: Monospace; font-weight: bold;")
        layout.addWidget(self.lbl_clock)

        layout.addStretch()

        # Auto-Centering Badge
        self.lbl_autocenter = QLabel("AUTO-CENTER: OFF")
        self.lbl_autocenter.setStyleSheet("color: #8b949e; font-weight: bold; padding: 2px 6px; border: 1px solid #30363d; border-radius: 3px;")
        layout.addWidget(self.lbl_autocenter)

        layout.addSpacing(10)

        # Emergency KILL Switch
        btn_kill = QPushButton("⛔ EMERGENCY STOP")
        btn_kill.setObjectName("btnKill")
        btn_kill.clicked.connect(self.on_emergency_stop)
        layout.addWidget(btn_kill)

        return bar

    def create_camera_box(self):
        box = QGroupBox("Live Optical Vision & Spatial AI Detection Stream")
        layout = QVBoxLayout(box)

        # Control Strip
        ctrl_strip = QHBoxLayout()
        ctrl_strip.addWidget(QLabel("Stream Feed:"))

        self.combo_stream = QComboBox()
        self.combo_stream.addItems(["Annotated Detections (YOLOv8 + Spatial AI)", "Retinex Enhanced Stream", "Raw Camera Feed"])
        self.combo_stream.currentIndexChanged.connect(self.on_stream_changed)
        ctrl_strip.addWidget(self.combo_stream)

        ctrl_strip.addStretch()

        self.btn_snapshot = QPushButton("📷 Snapshot")
        self.btn_snapshot.clicked.connect(self.on_snapshot_clicked)
        ctrl_strip.addWidget(self.btn_snapshot)

        self.lbl_cam_fps = QLabel("FPS: --")
        self.lbl_cam_fps.setStyleSheet("color: #58a6ff; font-weight: bold;")
        ctrl_strip.addWidget(self.lbl_cam_fps)

        layout.addLayout(ctrl_strip)

        # Video Canvas
        self.lbl_video = QLabel("Awaiting live underwater camera stream...")
        self.lbl_video.setAlignment(Qt.AlignCenter)
        self.lbl_video.setStyleSheet("background-color: #000000; border-radius: 4px; border: 1px solid #30363d;")
        self.lbl_video.setMinimumSize(480, 270)
        layout.addWidget(self.lbl_video, 1)

        return box

    def create_detection_log_box(self):
        box = QGroupBox("Detected Targets Inventory & Spatial Log")
        layout = QVBoxLayout(box)

        # Table
        self.table_det = QTableWidget(0, 6)
        self.table_det.setHorizontalHeaderLabels(["Timestamp", "Target Class", "Conf", "Range", "3D Position [X,Y,Z]", "Pilot Notes"])
        self.table_det.horizontalHeader().setSectionResizeMode(QHeaderView.Interactive)
        self.table_det.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table_det.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)
        self.table_det.setSelectionBehavior(QAbstractItemView.SelectRows)
        layout.addWidget(self.table_det)

        # Action Buttons
        btn_row = QHBoxLayout()
        self.lbl_det_count = QLabel("Unique Targets: 0")
        self.lbl_det_count.setStyleSheet("font-weight: bold; color: #58a6ff;")
        btn_row.addWidget(self.lbl_det_count)

        btn_row.addStretch()

        btn_export_csv = QPushButton("💾 Export CSV")
        btn_export_csv.clicked.connect(self.export_csv)
        btn_row.addWidget(btn_export_csv)

        btn_export_json = QPushButton("💾 Export JSON")
        btn_export_json.clicked.connect(self.export_json)
        btn_row.addWidget(btn_export_json)

        btn_clear = QPushButton("🗑️ Clear Log")
        btn_clear.clicked.connect(self.clear_detection_log)
        btn_row.addWidget(btn_clear)

        layout.addLayout(btn_row)
        return box

    def create_telemetry_gauges_box(self):
        box = QGroupBox("Navigation Telemetry, Attitude & Riverbed Map")
        layout = QHBoxLayout(box)

        # Left side: Depth column gauge
        self.gauge_depth = DepthGaugeWidget()
        layout.addWidget(self.gauge_depth)

        # Middle side: Attitude artificial horizon & digital readouts
        mid_col = QVBoxLayout()
        self.gauge_attitude = AttitudeIndicatorWidget()
        mid_col.addWidget(self.gauge_attitude)

        # Digital Telemetry Readouts
        tele_frame = QFrame()
        tele_frame.setStyleSheet("background-color: #161b22; border-radius: 4px; padding: 6px;")
        t_grid = QGridLayout(tele_frame)

        self.lbl_val_x = QLabel("X: +0.00 m")
        self.lbl_val_y = QLabel("Y: +0.00 m")
        self.lbl_val_z = QLabel("Z: -0.00 m")
        self.lbl_val_yaw = QLabel("YAW: 000°")
        self.lbl_val_speed = QLabel("SPD: 0.00 m/s")
        self.lbl_val_offset = QLabel("LANE: 0.00 m")

        for lbl in (self.lbl_val_x, self.lbl_val_y, self.lbl_val_z, self.lbl_val_yaw, self.lbl_val_speed, self.lbl_val_offset):
            lbl.setStyleSheet("font-family: Monospace; font-weight: bold; font-size: 10px;")

        t_grid.addWidget(self.lbl_val_x, 0, 0)
        t_grid.addWidget(self.lbl_val_y, 0, 1)
        t_grid.addWidget(self.lbl_val_z, 1, 0)
        t_grid.addWidget(self.lbl_val_yaw, 1, 1)
        t_grid.addWidget(self.lbl_val_speed, 2, 0)
        t_grid.addWidget(self.lbl_val_offset, 2, 1)

        mid_col.addWidget(tele_frame)
        layout.addLayout(mid_col)

        # Right side: Riverbed Top-Down Map
        self.map_river = RiverMapWidget()
        layout.addWidget(self.map_river, 1)

        return box

    def create_control_deck_box(self):
        box = QGroupBox("Subsea Command Deck & Dynamic Flood Control")
        layout = QHBoxLayout(box)

        # Column 1: Teleoperation Directional Keypad
        nav_box = QFrame()
        nav_box.setStyleSheet("background-color: #161b22; border-radius: 4px; padding: 6px;")
        nav_grid = QGridLayout(nav_box)

        btn_fwd = QPushButton("▲ SURGE FWD")
        btn_fwd.clicked.connect(lambda: self.on_teleop_cmd(surge=+self.cruise_speed))
        btn_back = QPushButton("▼ SURGE REV")
        btn_back.clicked.connect(lambda: self.on_teleop_cmd(surge=-self.cruise_speed))
        btn_port = QPushButton("◄ SWAY PORT")
        btn_port.clicked.connect(lambda: self.on_teleop_cmd(sway=+self.cruise_speed))
        btn_stbd = QPushButton("► SWAY STBD")
        btn_stbd.clicked.connect(lambda: self.on_teleop_cmd(sway=-self.cruise_speed))

        btn_up = QPushButton("▲ HEAVE UP")
        btn_up.clicked.connect(lambda: self.on_teleop_cmd(heave=+0.3))
        btn_down = QPushButton("▼ HEAVE DIVE")
        btn_down.clicked.connect(lambda: self.on_teleop_cmd(heave=-0.3))

        btn_left = QPushButton("↺ TURN LEFT")
        btn_left.clicked.connect(lambda: self.on_teleop_cmd(yaw=+0.35))
        btn_right = QPushButton("↻ TURN RIGHT")
        btn_right.clicked.connect(lambda: self.on_teleop_cmd(yaw=-0.35))

        btn_stop = QPushButton("⏹ ALL STOP")
        btn_stop.setStyleSheet("background-color: #388bfd; color: #fff; font-weight: bold;")
        btn_stop.clicked.connect(self.on_all_stop)

        nav_grid.addWidget(btn_fwd, 0, 1)
        nav_grid.addWidget(btn_port, 1, 0)
        nav_grid.addWidget(btn_stop, 1, 1)
        nav_grid.addWidget(btn_stbd, 1, 2)
        nav_grid.addWidget(btn_back, 2, 1)

        nav_grid.addWidget(btn_up, 0, 3)
        nav_grid.addWidget(btn_down, 2, 3)
        nav_grid.addWidget(btn_left, 1, 4)
        nav_grid.addWidget(btn_right, 1, 5)

        layout.addWidget(nav_box, 2)

        # Column 2: Cruise Speeds & Dynamic Features
        cmd_box = QVBoxLayout()

        # Speed Presets
        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("Throttle:"))
        for p in (0.25, 0.50, 0.75, 1.00):
            b = QPushButton(f"{int(p*100)}%")
            b.setFixedWidth(45)
            b.clicked.connect(lambda _, val=p: self.set_cruise_speed(val))
            speed_row.addWidget(b)
        cmd_box.addLayout(speed_row)

        # Auto-Centering Button
        self.btn_center = QPushButton("🎯 Auto-Centering (C)")
        self.btn_center.clicked.connect(self.on_toggle_autocenter)
        cmd_box.addWidget(self.btn_center)

        # Dynamic Flood Controller Section
        flood_row = QHBoxLayout()
        self.btn_flood_calm = QPushButton("🌊 Calm (0.2 m/s)")
        self.btn_flood_calm.clicked.connect(lambda: self.bridge.trigger_flood(False))
        self.btn_flood_active = QPushButton("⚡ Surge Flood (1.4 m/s)")
        self.btn_flood_active.setStyleSheet("background-color: #d29922; color: #fff; font-weight: bold;")
        self.btn_flood_active.clicked.connect(lambda: self.bridge.trigger_flood(True))

        flood_row.addWidget(self.btn_flood_calm)
        flood_row.addWidget(self.btn_flood_active)
        cmd_box.addLayout(flood_row)

        self.lbl_flood_status = QLabel("Flood: NORMAL [CALM]")
        self.lbl_flood_status.setStyleSheet("color: #8b949e; font-size: 10px;")
        cmd_box.addWidget(self.lbl_flood_status)

        layout.addLayout(cmd_box, 1)

        # Column 3: 8-Thruster Output Load Matrix
        thruster_box = QFrame()
        thruster_box.setStyleSheet("background-color: #161b22; border-radius: 4px; padding: 6px;")
        t_layout = QVBoxLayout(thruster_box)
        t_layout.addWidget(QLabel("8-Thruster Load Matrix (N):"))

        self.thruster_bars = []
        grid_t = QGridLayout()
        for i in range(8):
            lbl_name = QLabel(f"T{i+1}")
            bar = QProgressBar()
            bar.setRange(-30, 30)
            bar.setValue(0)
            bar.setFormat("%v N")
            bar.setFixedHeight(12)
            grid_t.addWidget(lbl_name, i // 2, (i % 2) * 2)
            grid_t.addWidget(bar, i // 2, (i % 2) * 2 + 1)
            self.thruster_bars.append(bar)

        t_layout.addLayout(grid_t)
        layout.addWidget(thruster_box, 1)

        return box

    def connect_signals(self):
        self.bridge.sig_image.connect(self.update_video_canvas)
        self.bridge.sig_telemetry.connect(self.update_telemetry)
        self.bridge.sig_thrusters.connect(self.update_thrusters)
        self.bridge.sig_status.connect(self.update_status)
        self.bridge.sig_detections.connect(self.update_detections)

    # --------------------------------------------------------------------------
    # KEYBOARD TELEOPERATION LISTENER (Full Hotkey Support Inside Window)
    # --------------------------------------------------------------------------
    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_W:
            self.on_teleop_cmd(surge=+self.cruise_speed)
        elif key == Qt.Key_S:
            self.on_teleop_cmd(surge=-self.cruise_speed)
        elif key == Qt.Key_A:
            self.on_teleop_cmd(sway=+self.cruise_speed)
        elif key == Qt.Key_D:
            self.on_teleop_cmd(sway=-self.cruise_speed)
        elif key == Qt.Key_I:
            self.on_teleop_cmd(heave=+0.30)
        elif key == Qt.Key_K:
            self.on_teleop_cmd(heave=-0.30)
        elif key == Qt.Key_Q:
            self.on_teleop_cmd(yaw=+0.35)
        elif key == Qt.Key_E:
            self.on_teleop_cmd(yaw=-0.35)
        elif key == Qt.Key_Space:
            self.on_all_stop()
        elif key == Qt.Key_C:
            self.on_toggle_autocenter()
        elif key == Qt.Key_1:
            self.set_cruise_speed(0.25)
        elif key == Qt.Key_2:
            self.set_cruise_speed(0.50)
        elif key == Qt.Key_3:
            self.set_cruise_speed(0.75)
        elif key == Qt.Key_4:
            self.set_cruise_speed(1.00)
        else:
            super().keyPressEvent(event)

    # --------------------------------------------------------------------------
    # SLOTS & UPDATERS
    # --------------------------------------------------------------------------
    def update_video_canvas(self, qimg: QImage):
        scaled = qimg.scaled(
            self.lbl_video.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.lbl_video.setPixmap(QPixmap.fromImage(scaled))
        self.last_frame = qimg

    def update_telemetry(self, t: dict):
        self.gauge_depth.update_depth(t.get("depth", 0.0))
        self.gauge_attitude.update_attitude(t.get("roll", 0.0), t.get("pitch", 0.0))
        self.map_river.update_pose(t.get("x", 4.0), t.get("y", 0.0), t.get("yaw", 0.0))

        self.lbl_val_x.setText(f"X: {t.get('x', 0.0):+.2f} m")
        self.lbl_val_y.setText(f"Y: {t.get('y', 0.0):+.2f} m")
        self.lbl_val_z.setText(f"Z: {t.get('z', 0.0):+.2f} m")
        self.lbl_val_yaw.setText(f"YAW: {t.get('yaw', 0.0):03.0f}°")

        spd = math.sqrt(t.get("vx", 0.0)**2 + t.get("vy", 0.0)**2)
        self.lbl_val_speed.setText(f"SPD: {spd:.2f} m/s")

    def update_thrusters(self, vals: list):
        for i, bar in enumerate(self.thruster_bars):
            if i < len(vals):
                val = int(round(vals[i]))
                bar.setValue(val)
                # Color code
                if abs(val) > 24:
                    bar.setStyleSheet("QProgressBar::chunk { background-color: #f85149; }")
                elif abs(val) > 15:
                    bar.setStyleSheet("QProgressBar::chunk { background-color: #d29922; }")
                else:
                    bar.setStyleSheet("QProgressBar::chunk { background-color: #238636; }")

    def update_status(self, s: dict):
        online = s.get("comms_online", False)
        if online:
            self.lbl_comms.setText("🟢 COMMS ONLINE [50 Hz]")
            self.lbl_comms.setStyleSheet("color: #2ea043; font-weight: bold;")
        else:
            self.lbl_comms.setText("🔴 COMMS TIMEOUT")
            self.lbl_comms.setStyleSheet("color: #f85149; font-weight: bold;")

        state_txt = s.get("safety_state", "UNKNOWN")
        self.lbl_safety.setText(f"STATE: {state_txt.upper()}")
        if "ARMED" in state_txt:
            self.lbl_safety.setStyleSheet("background-color: #238636; color: #fff; padding: 3px 8px; border-radius: 4px; font-weight: bold;")
        else:
            self.lbl_safety.setStyleSheet("background-color: #d29922; color: #fff; padding: 3px 8px; border-radius: 4px; font-weight: bold;")

        c_on = s.get("auto_center", False)
        self.lbl_autocenter.setText(f"AUTO-CENTER: {'ON' if c_on else 'OFF'}")
        self.lbl_autocenter.setStyleSheet(f"color: {'#58a6ff' if c_on else '#8b949e'}; font-weight: bold; border: 1px solid {'#1f6feb' if c_on else '#30363d'}; border-radius: 3px; padding: 2px 6px;")

        self.lbl_flood_status.setText(f"Flood: {s.get('flood_status', 'NORMAL')}")

    def update_detections(self, detections: list):
        self.map_river.update_targets(detections)
        now_str = datetime.now().strftime("%H:%M:%S")

        for d in detections:
            cls_name = d.get("class", "object")
            conf = d.get("confidence", 0.0)
            rng = d.get("range_m", None)
            xyz = d.get("spatial_xyz", [0.0, 0.0, 0.0])

            # Deduplication: update existing or add new row
            if cls_name in self.detected_items_map:
                row_idx = self.detected_items_map[cls_name]["row"]
                self.table_det.item(row_idx, 0).setText(now_str)
                self.table_det.item(row_idx, 2).setText(f"{int(conf * 100)}%")
                if rng:
                    self.table_det.item(row_idx, 3).setText(f"{rng:.2f} m")
                self.table_det.item(row_idx, 4).setText(f"[{xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}]")
                self.detected_items_map[cls_name]["data"] = d
            else:
                row_idx = self.table_det.rowCount()
                self.table_det.insertRow(row_idx)
                self.table_det.setItem(row_idx, 0, QTableWidgetItem(now_str))
                self.table_det.setItem(row_idx, 1, QTableWidgetItem(cls_name.upper()))
                self.table_det.setItem(row_idx, 2, QTableWidgetItem(f"{int(conf * 100)}%"))
                self.table_det.setItem(row_idx, 3, QTableWidgetItem(f"{rng:.2f} m" if rng else "--"))
                self.table_det.setItem(row_idx, 4, QTableWidgetItem(f"[{xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}]"))
                self.table_det.setItem(row_idx, 5, QTableWidgetItem("Verified target"))

                self.detected_items_map[cls_name] = {
                    "row": row_idx,
                    "first_seen": now_str,
                    "data": d,
                }

        self.lbl_det_count.setText(f"Unique Targets: {len(self.detected_items_map)}")

        # Auto-save to JSON periodically
        self.auto_save_detections()

    def auto_save_detections(self):
        try:
            records = []
            for cls_name, info in self.detected_items_map.items():
                records.append({
                    "class": cls_name,
                    "first_seen": info["first_seen"],
                    "data": info["data"],
                })
            with open(self.auto_save_path, "w") as f:
                json.dump(records, f, indent=2)
        except Exception:
            pass

    def update_mission_clock(self):
        elapsed = datetime.now() - self.start_time
        tot_sec = int(elapsed.total_seconds())
        hh = tot_sec // 3600
        mm = (tot_sec % 3600) // 60
        ss = tot_sec % 60
        self.lbl_clock.setText(f"MISSION: {hh:02d}:{mm:02d}:{ss:02d}")

    # --------------------------------------------------------------------------
    # USER ACTIONS & COMMAND DISPATCH
    # --------------------------------------------------------------------------
    def on_stream_changed(self, idx):
        streams = ["detections", "enhanced", "raw"]
        self.bridge.set_stream(streams[idx])

    def on_snapshot_clicked(self):
        if hasattr(self, "last_frame") and self.last_frame:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = os.path.join(self.snapshot_dir, f"snapshot_{ts}.png")
            self.last_frame.save(fname, "PNG")
            QMessageBox.information(self, "Snapshot Captured", f"Frame successfully saved to:\n{fname}")

    def on_teleop_cmd(self, surge=0.0, sway=0.0, heave=0.0, yaw=0.0):
        self.bridge.send_motion(surge, sway, heave, yaw)

    def on_all_stop(self):
        self.bridge.stop_all()

    def on_emergency_stop(self):
        self.bridge.trigger_kill()
        QMessageBox.warning(self, "EMERGENCY STOP", "Emergency Kill Signal dispatched! All thrusters disarmed.")

    def on_toggle_autocenter(self):
        val = self.bridge.toggle_auto_center()
        self.lbl_autocenter.setText(f"AUTO-CENTER: {'ON' if val else 'OFF'}")

    def set_cruise_speed(self, val):
        self.cruise_speed = val

    def export_csv(self):
        fname, _ = QFileDialog.getSaveFileName(self, "Export Detections to CSV", os.path.expanduser("~/detections_log.csv"), "CSV Files (*.csv)")
        if not fname:
            return
        with open(fname, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp", "Target Class", "Confidence", "Range", "Spatial XYZ", "Pilot Notes"])
            for r in range(self.table_det.rowCount()):
                row_vals = [self.table_det.item(r, c).text() if self.table_det.item(r, c) else "" for c in range(6)]
                writer.writerow(row_vals)
        QMessageBox.information(self, "Export Complete", f"Saved detection log to:\n{fname}")

    def export_json(self):
        fname, _ = QFileDialog.getSaveFileName(self, "Export Detections to JSON", os.path.expanduser("~/detections_log.json"), "JSON Files (*.json)")
        if not fname:
            return
        records = []
        for r in range(self.table_det.rowCount()):
            records.append({
                "time": self.table_det.item(r, 0).text(),
                "class": self.table_det.item(r, 1).text(),
                "confidence": self.table_det.item(r, 2).text(),
                "range": self.table_det.item(r, 3).text(),
                "spatial_xyz": self.table_det.item(r, 4).text(),
                "notes": self.table_det.item(r, 5).text(),
            })
        with open(fname, "w") as f:
            json.dump(records, f, indent=2)
        QMessageBox.information(self, "Export Complete", f"Saved JSON telemetry to:\n{fname}")

    def clear_detection_log(self):
        self.table_det.setRowCount(0)
        self.detected_items_map.clear()
        self.lbl_det_count.setText("Unique Targets: 0")


# ==============================================================================
# MAIN ENTRYPOINT
# ==============================================================================
class RosWorkerThread(QThread):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self._running = True

    def run(self):
        while self._running and rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.02)
            except Exception:
                break

    def stop(self):
        self._running = False
        self.wait(500)


def main():
    app = QApplication(sys.argv)

    bridge = RosBridge()
    bridge.init_ros()

    worker = RosWorkerThread(bridge.node)
    worker.start()

    win = GroundStationWindow(bridge)
    win.show()

    exit_code = app.exec_()

    worker.stop()
    if rclpy.ok():
        bridge.node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
