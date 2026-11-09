"""Pilot's ground station for isonavi-rov.

Shows the video streams, depth, attitude, thruster loads and the supervisor
state, and drives the vehicle from the keypad or keyboard. The plan view uses
the simulator's odometry, which the vehicle has no equivalent of.

It publishes joy at 20 Hz, so it is the pilot link: stop it and the supervisor
holds. Run with -p drive:=false to watch alongside a gamepad or keyboard_teleop.
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

    def __init__(self):
        super().__init__()
        self.node = None
        self.bridge = CvBridge()
        self.active_stream = "detections"
        self.latest_turbid_cv = None
        self.latest_det_cv = None
        self.latest_raw_cv = None

        # Command states
        self.surge = 0.0
        self.sway = 0.0
        self.heave = 0.0
        self.yaw = 0.0
        self.auto_center = False
        self.press_ticks = 0    # the pilot toggles centring on a rising edge
        self.last_supervisor_t = 0.0

    def init_ros(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("isonavi_gcs_node")
        self.drive = self.node.declare_parameter("drive", True).value
        self.world = self.node.declare_parameter("world", "pool_course").value

        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
        )

        # Subscriptions
        self.sub_img_det = self.node.create_subscription(
            Image, "camera/image_detections", self.on_image_det, qos_best_effort
        )
        self.sub_img_enh = self.node.create_subscription(
            Image, "camera/image_enhanced", self.on_image_enh, qos_best_effort
        )
        self.sub_img_turbid = self.node.create_subscription(
            Image, "camera/image_raw_turbid", self.on_image_turbid, qos_best_effort
        )
        self.sub_img_raw = self.node.create_subscription(
            Image, "camera/image_raw", self.on_image_raw, qos_best_effort
        )
        self.sub_det_json = self.node.create_subscription(
            String, "camera/detections", self.on_detections, 10
        )
        self.sub_offset = self.node.create_subscription(
            Float64, "corridor_offset", self.on_offset, 10
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

        # Telemetry Cache
        self.telemetry = {
            "depth": 0.0,
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
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
            "flood_status": "CALM",
            "comms_online": False,
            "auto_center": False,
        }

        # 20 Hz Command/Heartbeat Timer
        self.timer_joy = self.node.create_timer(0.05, self.publish_joy)

    def set_stream(self, stream_type: str):
        self.active_stream = stream_type

    def on_image_det(self, msg: Image):
        try:
            self.latest_det_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass
        if self.active_stream == "detections":
            self.process_image(msg)
        elif self.active_stream == "split":
            self.generate_split_view()

    def on_image_enh(self, msg: Image):
        if self.active_stream == "enhanced":
            self.process_image(msg)

    def on_image_turbid(self, msg: Image):
        try:
            self.latest_turbid_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass
        if self.active_stream == "turbid":
            self.process_image(msg)
        elif self.active_stream == "split":
            self.generate_split_view()

    def on_image_raw(self, msg: Image):
        try:
            self.latest_raw_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass
        if self.active_stream == "raw":
            self.process_image(msg)
        elif self.active_stream == "turbid" and self.latest_turbid_cv is None:
            self.process_image(msg)
        elif self.active_stream == "split" and self.latest_turbid_cv is None:
            self.generate_split_view()

    def generate_split_view(self):
        left_bgr = self.latest_turbid_cv if self.latest_turbid_cv is not None else self.latest_raw_cv
        right_bgr = self.latest_det_cv
        if left_bgr is None and right_bgr is None:
            return
        if left_bgr is None:
            self.emit_cv_image(right_bgr)
            return
        if right_bgr is None:
            self.emit_cv_image(left_bgr)
            return

        h, w = right_bgr.shape[:2]
        if left_bgr.shape[:2] != (h, w):
            left_bgr = cv2.resize(left_bgr, (w, h))

        mid_x = w // 2
        split = np.empty_like(right_bgr)
        split[:, :mid_x] = left_bgr[:, :mid_x]
        split[:, mid_x:] = right_bgr[:, mid_x:]

        # Center vertical divider line (amber gold #e5c07b)
        cv2.line(split, (mid_x, 0), (mid_x, h), (123, 192, 229), 2)

        cv2.rectangle(split, (8, 8), (242, 30), (18, 18, 18), -1)
        cv2.putText(split, "CAMERA", (14, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 200, 220), 1, cv2.LINE_AA)

        cv2.rectangle(split, (mid_x + 8, 8), (mid_x + 252, 30), (18, 18, 18), -1)
        cv2.putText(split, "DEHAZED", (mid_x + 14, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 120), 1, cv2.LINE_AA)

        # Centered Dual Split-View Badge
        (bw, bh), _ = cv2.getTextSize("DUAL SPLIT-VIEW COMPARISON", cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
        bx = (w - bw) // 2
        cv2.rectangle(split, (bx - 6, h - 24), (bx + bw + 6, h - 6), (18, 18, 18), -1)
        cv2.putText(split, "DUAL SPLIT-VIEW COMPARISON", (bx, h - 11), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (123, 192, 229), 1, cv2.LINE_AA)

        self.emit_cv_image(split)

    def emit_cv_image(self, cv_img: np.ndarray):
        try:
            rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb.shape
            bytes_per_line = ch * w
            qimg = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888).copy()
            self.sig_image.emit(qimg)
        except Exception:
            pass

    def process_image(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.emit_cv_image(cv_img)
        except Exception:
            pass

    def on_offset(self, msg: Float64):
        self.telemetry["offset"] = float(msg.data)

    def on_depth(self, msg: Float64):
        self.telemetry["depth"] = float(msg.data)
        self.sig_telemetry.emit(dict(self.telemetry))

    def on_odom(self, msg: Odometry):
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
        r, pt, y = quat_to_euler(msg.orientation)
        self.telemetry["roll"] = r
        self.telemetry["pitch"] = pt
        self.telemetry["yaw"] = y
        self.sig_telemetry.emit(dict(self.telemetry))

    def on_thrusters(self, msg: Float64MultiArray):
        self.sig_thrusters.emit(list(msg.data))

    def on_safety(self, msg: String):
        self.last_supervisor_t = time.time()
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
        msg.buttons = [1 if self.press_ticks > 0 else 0, 0, 0, 0, 0, 0, 0, 0]
        self.press_ticks = max(0, self.press_ticks - 1)
        if self.drive:
            self.pub_joy.publish(msg)

        # The link is up while the vehicle's supervisor is reporting
        self.status["comms_online"] = time.time() - self.last_supervisor_t < 1.2
        self.status["auto_center"] = self.auto_center
        self.sig_status.emit(dict(self.status))

    def send_motion(self, surge, sway, heave, yaw):
        self.surge = float(surge)
        self.sway = float(sway)
        self.heave = float(heave)
        self.yaw = float(yaw)

    def toggle_auto_center(self):
        self.auto_center = not self.auto_center
        self.press_ticks = 2
        self.status["auto_center"] = self.auto_center
        self.sig_status.emit(dict(self.status))
        return self.auto_center

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
# PROFESSIONAL INSTRUMENT GAUGES
# ==============================================================================
class AttitudeIndicatorWidget(QWidget):
    """Primary Flight Display (PFD) Artificial Horizon with Calibrated Roll/Pitch Scale."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(150, 150)
        self.roll = 0.0
        self.pitch = 0.0

    def update_attitude(self, roll, pitch):
        self.roll = roll
        self.pitch = pitch
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        cx, cy = w / 2.0, h / 2.0
        rad = min(cx, cy) - 6

        # Instrument Border
        p.fillRect(0, 0, w, h, QColor("#181a1f"))

        # Circular horizon mask
        clip = QPainterPath()
        clip.addEllipse(QPointF(cx, cy), rad, rad)
        p.setClipPath(clip)

        # Transformed Sky & Ground
        p.save()
        p.translate(cx, cy)
        p.rotate(-self.roll)

        pitch_px = max(-rad * 1.5, min(rad * 1.5, self.pitch * 1.8))

        # Sky (Aviation Dark Blue) & Ground (Marine Silt Olive-Brown)
        p.fillRect(QRectF(-rad * 2, -rad * 2.5 + pitch_px, rad * 4, rad * 2.5), QColor("#1c3b5e"))
        p.fillRect(QRectF(-rad * 2, pitch_px, rad * 4, rad * 2.5), QColor("#362f26"))

        # Zero Horizon Line
        p.setPen(QPen(QColor("#ffffff"), 1.8))
        p.drawLine(QPointF(-rad * 1.5, pitch_px), QPointF(rad * 1.5, pitch_px))

        # Pitch Ladder Rungs
        p.setFont(QFont("Monospace", 7))
        p.setPen(QPen(QColor("#ffffff"), 1.2))
        for deg in (-30, -20, -10, 10, 20, 30):
            ry = pitch_px - deg * 1.8
            p.drawLine(QPointF(-18, ry), QPointF(-6, ry))
            p.drawLine(QPointF(6, ry), QPointF(18, ry))
            p.drawText(QPointF(20, ry + 3), f"{abs(deg)}")
            p.drawText(QPointF(-32, ry + 3), f"{abs(deg)}")

        p.restore()
        p.setClipping(False)

        # Fixed Aircraft / ROV Reference Symbol
        p.setPen(QPen(QColor("#e5c07b"), 2.2))
        p.drawLine(int(cx - 28), int(cy), int(cx - 10), int(cy))
        p.drawLine(int(cx + 10), int(cy), int(cx + 28), int(cy))
        p.drawLine(int(cx - 10), int(cy), int(cx - 10), int(cy + 6))
        p.drawLine(int(cx + 10), int(cy), int(cx + 10), int(cy + 6))
        p.drawEllipse(QPointF(cx, cy), 3, 3)

        # Outer Instrument Bezel
        p.setPen(QPen(QColor("#3b4048"), 2.0))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(cx, cy), rad, rad)

        # Roll Pointer Index
        p.setPen(QPen(QColor("#e5c07b"), 2))
        p.drawLine(int(cx), int(cy - rad), int(cx), int(cy - rad + 8))

        # Subtitle readout
        p.setFont(QFont("Monospace", 8, QFont.Bold))
        p.setPen(QColor("#abb2bf"))
        p.drawText(QRectF(0, h - 16, w, 14), Qt.AlignCenter, f"R:{self.roll:+.1f}°  P:{self.pitch:+.1f}°")


class DepthGaugeWidget(QWidget):
    """Calibrated Subsea Vertical Depth Tape with Digital Readout."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(85, 180)
        self.depth_m = 0.0

    def update_depth(self, d):
        self.depth_m = max(0.0, d)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor("#181a1f"))

        bar_x = 18
        bar_w = 20
        bar_y = 28
        bar_h = h - 56

        max_scale = 5.0  # 5 meters scale
        clamped_d = min(max_scale, self.depth_m)
        fill_h = (clamped_d / max_scale) * bar_h

        # Column background
        p.fillRect(bar_x, bar_y, bar_w, bar_h, QColor("#121417"))

        # Water column fill
        p.fillRect(bar_x, bar_y, bar_w, int(fill_h), QColor("#224b73"))

        # Border
        p.setPen(QPen(QColor("#3b4048"), 1.2))
        p.drawRect(bar_x, bar_y, bar_w, bar_h)

        # Ticks and graduation marks
        p.setFont(QFont("Monospace", 7))
        p.setPen(QColor("#7f848e"))
        for m in range(0, 6):
            ty = bar_y + (m / max_scale) * bar_h
            p.drawLine(bar_x + bar_w, int(ty), bar_x + bar_w + 5, int(ty))
            p.drawText(bar_x + bar_w + 7, int(ty + 3), f"{m}.0m")

        # Current Depth Target Pointer
        ind_y = bar_y + fill_h
        p.setPen(QPen(QColor("#e06c75"), 2))
        p.drawLine(bar_x - 4, int(ind_y), bar_x + bar_w + 4, int(ind_y))

        # Title and digital box
        p.setFont(QFont("Monospace", 8, QFont.Bold))
        p.setPen(QColor("#61afef"))
        p.drawText(QRectF(0, 8, w, 16), Qt.AlignCenter, "DEPTH (M)")

        p.setFont(QFont("Monospace", 9, QFont.Bold))
        p.setPen(QColor("#ffffff"))
        p.drawText(QRectF(0, h - 22, w, 18), Qt.AlignCenter, f"{self.depth_m:.2f} m")


# x from, x to, half width in metres, and whether the large river's structures apply
MAP_EXTENTS = {
    "pool_course": (0.0, 20.0, 5.0, False),
    "indian_river": (-2.5, 32.5, 9.0, False),
    "large_river": (0.0, 80.0, 8.0, True),
}


class RiverMapWidget(QWidget):
    """Plan view from the simulator's odometry."""
    def __init__(self, world="pool_course", parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 220)
        self.x0, self.x1, self.half, self.landmarks = MAP_EXTENTS.get(world, MAP_EXTENTS["pool_course"])
        self.rov_x = 0.0
        self.rov_y = 0.0
        self.rov_yaw = 0.0
        self.trail = deque(maxlen=240)
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

        p.fillRect(0, 0, w, h, QColor("#14161a"))

        pad_x = 30
        pad_y = 22
        map_w = w - 2 * pad_x
        map_h = h - 2 * pad_y

        def world_to_screen(wx, wy):
            sx = pad_x + ((wx - self.x0) / (self.x1 - self.x0)) * map_w
            sy = h / 2.0 - (wy / self.half) * (map_h / 2.0)
            return sx, sy

        # Grid lines
        p.setFont(QFont("Monospace", 7))
        p.setPen(QPen(QColor("#242830"), 1, Qt.DotLine))
        for gx in np.linspace(self.x0, self.x1, 5):
            sx, _ = world_to_screen(gx, 0)
            p.drawLine(int(sx), int(pad_y), int(sx), int(h - pad_y))
            p.setPen(QColor("#5c6370"))
            p.drawText(int(sx - 10), int(h - 8), f"{int(gx)}m")
            p.setPen(QPen(QColor("#242830"), 1, Qt.DotLine))

        for gy in (-0.75 * self.half, 0.0, 0.75 * self.half):
            _, sy = world_to_screen(0, gy)
            p.drawLine(int(pad_x), int(sy), int(w - pad_x), int(sy))

        # Channel boundaries
        y_top_s = world_to_screen(0, self.half)[1]
        y_bot_s = world_to_screen(0, -self.half)[1]
        p.setPen(QPen(QColor("#3b4048"), 1.2))
        p.drawLine(int(pad_x), int(y_top_s), int(w - pad_x), int(y_top_s))
        p.drawLine(int(pad_x), int(y_bot_s), int(w - pad_x), int(y_bot_s))

        # Water channel subtle shading
        p.fillRect(QRectF(pad_x, y_top_s, map_w, y_bot_s - y_top_s), QColor(20, 32, 48, 120))

        # Structures in the large river
        if self.landmarks:
            p.setFont(QFont("Monospace", 7, QFont.Bold))
            # Pier 1
            p1_x, p1_y = world_to_screen(22.0, 0.0)
            p.setPen(QPen(QColor("#61afef"), 1.2))
            p.setBrush(QColor("#1e3b5e"))
            p.drawRect(QRectF(p1_x - 6, p1_y - 6, 12, 12))
            p.drawText(int(p1_x - 16), int(p1_y - 10), "PIER 1")

            # Pipeline
            pipe_x = world_to_screen(42.0, 0.0)[0]
            p.setPen(QPen(QColor("#e5c07b"), 1.5, Qt.DashLine))
            p.drawLine(int(pipe_x), int(y_top_s), int(pipe_x), int(y_bot_s))
            p.drawText(int(pipe_x - 14), int(y_bot_s + 12), "PIPELINE")

            # Pier 2
            p2_x, p2_y = world_to_screen(58.0, 0.0)
            p.setPen(QPen(QColor("#61afef"), 1.2))
            p.setBrush(QColor("#1e3b5e"))
            p.drawRect(QRectF(p2_x - 6, p2_y - 6, 12, 12))
            p.drawText(int(p2_x - 16), int(p2_y - 10), "PIER 2")

        # Historical Trajectory Trail
        if len(self.trail) > 1:
            p.setPen(QPen(QColor("#4078c0"), 1.5))
            pts = [QPointF(*world_to_screen(tx, ty)) for tx, ty in self.trail]
            for i in range(len(pts) - 1):
                p.drawLine(pts[i], pts[i + 1])

        # Contacts: camera frame x right, z forward, turned by the heading
        heading = math.radians(self.rov_yaw)
        for t in self.targets:
            if "spatial_xyz" in t:
                right, _, fwd = t["spatial_xyz"]
                tx_w = self.rov_x + fwd * math.cos(heading) + right * math.sin(heading)
                ty_w = self.rov_y + fwd * math.sin(heading) - right * math.cos(heading)
                tsx, tsy = world_to_screen(tx_w, ty_w)
                p.setPen(QPen(QColor("#e06c75"), 1.2))
                p.setBrush(QColor("#9c3a44"))
                p.drawPolygon(QPolygonF([
                    QPointF(tsx, tsy - 5),
                    QPointF(tsx + 5, tsy),
                    QPointF(tsx, tsy + 5),
                    QPointF(tsx - 5, tsy),
                ]))
                cls_txt = t.get("class", "TGT").split("_")[-1].upper()[:4]
                p.setPen(QColor("#e5c07b"))
                p.drawText(int(tsx + 6), int(tsy + 3), cls_txt)

        # ROV Vector & Icon
        rx, ry = world_to_screen(self.rov_x, self.rov_y)
        yaw_rad = math.radians(self.rov_yaw)

        # Velocity Leader Line
        p.setPen(QPen(QColor("#98c379"), 2))
        vx = rx + 18.0 * math.cos(yaw_rad)
        vy = ry - 18.0 * math.sin(yaw_rad)
        p.drawLine(QPointF(rx, ry), QPointF(vx, vy))

        # Triangle symbol
        p.setPen(QPen(QColor("#ffffff"), 1.4))
        p.setBrush(QColor("#2d8a4e"))
        tip = QPointF(rx + 9 * math.cos(yaw_rad), ry - 9 * math.sin(yaw_rad))
        p1 = QPointF(rx + 6 * math.cos(yaw_rad + 2.5), ry - 6 * math.sin(yaw_rad + 2.5))
        p2 = QPointF(rx + 6 * math.cos(yaw_rad - 2.5), ry - 6 * math.sin(yaw_rad - 2.5))
        p.drawPolygon(QPolygonF([tip, p1, p2]))

        # Telemetry Legend
        p.setFont(QFont("Monospace", 8))
        p.setPen(QColor("#abb2bf"))
        p.drawText(pad_x, 15, f"SIM POS: X={self.rov_x:.2f}m  Y={self.rov_y:.2f}m  HDG={self.rov_yaw:.0f}°")


# ==============================================================================
# MAIN OPERATIONS CONSOLE WINDOW
# ==============================================================================
class GroundStationWindow(QMainWindow):
    def __init__(self, bridge: RosBridge):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle("isonavi-rov ground station")
        self.resize(1440, 920)

        # Internal state
        self.start_time = datetime.now()
        self.cruise_speed = 0.50
        self.detected_items_map = {}
        self.auto_save_path = os.path.expanduser("~/.ros/isonavi_detection_log.json")
        self.snapshot_dir = os.path.expanduser("~/isonavi_snapshots")
        os.makedirs(self.snapshot_dir, exist_ok=True)

        self.apply_theme()
        self.init_ui()
        self.connect_signals()

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self.update_clock)
        self.ui_timer.start(100)

    def apply_theme(self):
        """Engineering Dark Palette (Slate / Charcoal / Tactical Muted Accents)."""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #14161a;
            }
            QWidget {
                background-color: #14161a;
                color: #abb2bf;
                font-family: 'Segoe UI', 'Ubuntu', 'DejaVu Sans', sans-serif;
            }
            QGroupBox {
                border: 1px solid #282c34;
                border-radius: 4px;
                margin-top: 14px;
                font-weight: 600;
                font-size: 10px;
                letter-spacing: 0.8px;
                padding-top: 8px;
                color: #61afef;
                background-color: #181a1f;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
                background-color: #14161a;
            }
            QPushButton {
                background-color: #21252b;
                border: 1px solid #3b4048;
                border-radius: 3px;
                padding: 5px 10px;
                color: #c8ccd4;
                font-size: 11px;
                font-weight: 500;
            }
            QPushButton:hover {
                background-color: #282c34;
                border-color: #5c6370;
            }
            QPushButton:pressed {
                background-color: #3e4451;
                color: #ffffff;
            }
            QPushButton#btnKill {
                background-color: #b33939;
                border-color: #cd3d3d;
                color: #ffffff;
                font-weight: bold;
                letter-spacing: 0.5px;
            }
            QPushButton#btnKill:hover {
                background-color: #d63031;
            }
            QTableWidget {
                background-color: #181a1f;
                border: 1px solid #282c34;
                gridline-color: #21252b;
                selection-background-color: #2c313a;
                selection-color: #ffffff;
                font-family: 'JetBrains Mono', 'Fira Code', 'DejaVu Sans Mono', monospace;
                font-size: 11px;
            }
            QHeaderView::section {
                background-color: #21252b;
                color: #7f848e;
                padding: 4px 6px;
                border: 1px solid #282c34;
                font-weight: 600;
                font-size: 10px;
            }
            QLabel {
                font-size: 11px;
            }
            QProgressBar {
                background-color: #121417;
                border: 1px solid #282c34;
                border-radius: 2px;
                text-align: center;
                color: #d7dae0;
                font-size: 8px;
                font-family: Monospace;
            }
            QProgressBar::chunk {
                background-color: #2d8a4e;
            }
            QComboBox {
                background-color: #21252b;
                border: 1px solid #3b4048;
                border-radius: 3px;
                padding: 3px 8px;
                color: #abb2bf;
                font-size: 11px;
            }
        """)

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(8, 6, 8, 6)
        main_layout.setSpacing(6)

        # 1. Institutional Top Navigation & Telemetry Ribbon
        main_layout.addWidget(self.create_header_ribbon())

        # 2. Main Console Splitter (Left: Optical Vision & Target DB, Right: PFD, Map, Thrusters, Command Deck)
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter, 1)

        # Left Column: Optical Vision Viewport & Classification Inventory
        left_col = QWidget()
        left_layout = QVBoxLayout(left_col)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(6)
        left_layout.addWidget(self.create_vision_viewport_box(), 3)
        left_layout.addWidget(self.create_target_inventory_box(), 2)
        splitter.addWidget(left_col)

        # Right column: instruments, plan view, controls
        right_col = QWidget()
        right_layout = QVBoxLayout(right_col)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(6)
        right_layout.addWidget(self.create_nav_telemetry_box(), 2)
        right_layout.addWidget(self.create_subsea_deck_box(), 2)
        splitter.addWidget(right_col)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

    def create_header_ribbon(self):
        frame = QFrame()
        frame.setStyleSheet("background-color: #181a1f; border: 1px solid #282c34; border-radius: 4px;")
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(12, 6, 12, 6)

        # Title
        title_box = QVBoxLayout()
        title_box.setSpacing(1)
        lbl_inst = QLabel("isonavi-rov")
        lbl_inst.setStyleSheet("font-size: 15px; font-weight: bold; color: #61afef; letter-spacing: 1px;")
        lbl_sub = QLabel("Ground station")
        lbl_sub.setStyleSheet("font-size: 10px; color: #5c6370; font-weight: 500;")
        title_box.addWidget(lbl_inst)
        title_box.addWidget(lbl_sub)
        layout.addLayout(title_box)

        layout.addSpacing(25)

        # Link state
        self.lbl_comms = QLabel("LINK: WAITING")
        self.lbl_comms.setStyleSheet("color: #98c379; font-family: Monospace; font-weight: 600; font-size: 11px;")
        layout.addWidget(self.lbl_comms)

        layout.addSpacing(15)

        # Safety Supervisor State Badge
        self.lbl_safety = QLabel("SUPERVISOR: --")
        self.lbl_safety.setStyleSheet("background-color: #238636; color: #ffffff; padding: 3px 8px; border-radius: 3px; font-weight: 600; font-size: 10px; letter-spacing: 0.5px;")
        layout.addWidget(self.lbl_safety)

        layout.addSpacing(15)

        # Mission Elapsed Clock
        self.lbl_clock = QLabel("MISSION TIME: 00:00:00")
        self.lbl_clock.setStyleSheet("color: #7f848e; font-family: Monospace; font-weight: 600; font-size: 11px;")
        layout.addWidget(self.lbl_clock)

        layout.addStretch()

        # Lane Centring Indicator
        self.lbl_autocenter = QLabel("LANE CENTRING: STANDBY")
        self.lbl_autocenter.setStyleSheet("color: #7f848e; font-family: Monospace; font-size: 10px; padding: 2px 6px; border: 1px solid #3b4048; border-radius: 2px;")
        layout.addWidget(self.lbl_autocenter)

        layout.addSpacing(12)

        # Emergency Disarm / KILL Button
        btn_kill = QPushButton("EMERGENCY DISARM [KILL]")
        btn_kill.setObjectName("btnKill")
        btn_kill.clicked.connect(self.on_emergency_stop)
        layout.addWidget(btn_kill)

        return frame

    def create_vision_viewport_box(self):
        box = QGroupBox("VIDEO")
        layout = QVBoxLayout(box)

        # Top Stream Configuration Bar
        ctrl_bar = QHBoxLayout()
        ctrl_bar.addWidget(QLabel("Video Channel:"))

        self.combo_stream = QComboBox()
        self.combo_stream.addItems([
            "Detections (/camera/image_detections)",
            "Dehazed (/camera/image_enhanced)",
            "Simulated mud (/camera/image_raw_turbid)",
            "Split: camera and detections",
            "Camera (/camera/image_raw)",
        ])
        self.combo_stream.currentIndexChanged.connect(self.on_stream_changed)
        ctrl_bar.addWidget(self.combo_stream)

        ctrl_bar.addStretch()

        btn_snap = QPushButton("Capture Frame [PNG]")
        btn_snap.clicked.connect(self.on_snapshot_clicked)
        ctrl_bar.addWidget(btn_snap)

        self.lbl_fps = QLabel("FPS: --")
        self.fps = 0.0
        self.last_frame_t = None
        self.lbl_fps.setStyleSheet("color: #61afef; font-family: Monospace; font-weight: 600;")
        ctrl_bar.addWidget(self.lbl_fps)

        layout.addLayout(ctrl_bar)

        # Video Frame Canvas
        self.lbl_video = QLabel("Awaiting live subsea camera feed...")
        self.lbl_video.setAlignment(Qt.AlignCenter)
        self.lbl_video.setStyleSheet("background-color: #0d0f12; border: 1px solid #282c34; border-radius: 3px;")
        self.lbl_video.setMinimumSize(520, 290)
        layout.addWidget(self.lbl_video, 1)

        return box

    def create_target_inventory_box(self):
        box = QGroupBox("CAMERA CONTACTS")
        layout = QVBoxLayout(box)

        # Target Table
        self.table_det = QTableWidget(0, 6)
        self.table_det.setHorizontalHeaderLabels([
            "TIME (UTC)", "OBJECT CLASSIFICATION", "CONFIDENCE", "SLANT RANGE", "EST. 3D POSITION (X, Y, Z)", "OPERATOR LOG"
        ])
        self.table_det.horizontalHeader().setSectionResizeMode(QHeaderView.Interactive)
        self.table_det.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table_det.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)
        self.table_det.setSelectionBehavior(QAbstractItemView.SelectRows)
        layout.addWidget(self.table_det)

        # Action Bar
        action_bar = QHBoxLayout()
        self.lbl_det_count = QLabel("Classified Contacts: 0")
        self.lbl_det_count.setStyleSheet("color: #61afef; font-weight: 600; font-family: Monospace;")
        action_bar.addWidget(self.lbl_det_count)

        action_bar.addStretch()

        btn_csv = QPushButton("Export Inspection Log [CSV]")
        btn_csv.clicked.connect(self.export_csv)
        action_bar.addWidget(btn_csv)

        btn_json = QPushButton("Export Telemetry [JSON]")
        btn_json.clicked.connect(self.export_json)
        action_bar.addWidget(btn_json)

        btn_clear = QPushButton("Clear Table")
        btn_clear.clicked.connect(self.clear_detection_log)
        action_bar.addWidget(btn_clear)

        layout.addLayout(action_bar)
        return box

    def create_nav_telemetry_box(self):
        box = QGroupBox("INSTRUMENTS AND PLAN VIEW")
        layout = QHBoxLayout(box)

        # Primary Flight Instruments: Depth Tape & Attitude PFD
        self.gauge_depth = DepthGaugeWidget()
        layout.addWidget(self.gauge_depth)

        mid_pane = QVBoxLayout()
        self.gauge_attitude = AttitudeIndicatorWidget()
        mid_pane.addWidget(self.gauge_attitude)

        # Digital Telemetry Block
        data_frame = QFrame()
        data_frame.setStyleSheet("background-color: #121417; border: 1px solid #282c34; border-radius: 3px; padding: 4px;")
        d_grid = QGridLayout(data_frame)

        self.lbl_val_x = QLabel("X: +0.00 m")
        self.lbl_val_y = QLabel("Y: +0.00 m")
        self.lbl_val_z = QLabel("Z: -0.00 m")
        self.lbl_val_yaw = QLabel("HDG: 000°")
        self.lbl_val_speed = QLabel("SPD: 0.00 m/s")
        self.lbl_val_offset = QLabel("LANE: --")

        for lbl in (self.lbl_val_x, self.lbl_val_y, self.lbl_val_z, self.lbl_val_yaw, self.lbl_val_speed, self.lbl_val_offset):
            lbl.setStyleSheet("font-family: 'JetBrains Mono', 'Fira Code', Monospace; font-weight: 600; font-size: 10px; color: #abb2bf;")

        d_grid.addWidget(self.lbl_val_x, 0, 0)
        d_grid.addWidget(self.lbl_val_y, 0, 1)
        d_grid.addWidget(self.lbl_val_z, 1, 0)
        d_grid.addWidget(self.lbl_val_yaw, 1, 1)
        d_grid.addWidget(self.lbl_val_speed, 2, 0)
        d_grid.addWidget(self.lbl_val_offset, 2, 1)

        mid_pane.addWidget(data_frame)
        layout.addLayout(mid_pane)

        self.map_river = RiverMapWidget(self.bridge.world)
        layout.addWidget(self.map_river, 1)

        return box

    def create_subsea_deck_box(self):
        box = QGroupBox("THRUSTERS AND COMMANDS")
        layout = QHBoxLayout(box)

        # 1. Teleoperation Keypad
        deck_frame = QFrame()
        deck_frame.setStyleSheet("background-color: #121417; border: 1px solid #282c34; border-radius: 3px; padding: 4px;")
        grid = QGridLayout(deck_frame)

        btn_fwd = QPushButton("SURGE +")
        btn_fwd.clicked.connect(lambda: self.on_teleop_cmd(surge=+self.cruise_speed))
        btn_back = QPushButton("SURGE -")
        btn_back.clicked.connect(lambda: self.on_teleop_cmd(surge=-self.cruise_speed))
        btn_port = QPushButton("SWAY L")
        btn_port.clicked.connect(lambda: self.on_teleop_cmd(sway=+self.cruise_speed))
        btn_stbd = QPushButton("SWAY R")
        btn_stbd.clicked.connect(lambda: self.on_teleop_cmd(sway=-self.cruise_speed))

        btn_up = QPushButton("HEAVE +")
        btn_up.clicked.connect(lambda: self.on_teleop_cmd(heave=+0.3))
        btn_down = QPushButton("HEAVE -")
        btn_down.clicked.connect(lambda: self.on_teleop_cmd(heave=-0.3))

        btn_left = QPushButton("YAW L")
        btn_left.clicked.connect(lambda: self.on_teleop_cmd(yaw=+0.35))
        btn_right = QPushButton("YAW R")
        btn_right.clicked.connect(lambda: self.on_teleop_cmd(yaw=-0.35))

        btn_stop = QPushButton("ALL STOP")
        btn_stop.setStyleSheet("background-color: #2b5b84; color: #ffffff; font-weight: bold;")
        btn_stop.clicked.connect(self.on_all_stop)

        grid.addWidget(btn_fwd, 0, 1)
        grid.addWidget(btn_port, 1, 0)
        grid.addWidget(btn_stop, 1, 1)
        grid.addWidget(btn_stbd, 1, 2)
        grid.addWidget(btn_back, 2, 1)

        grid.addWidget(btn_up, 0, 3)
        grid.addWidget(btn_down, 2, 3)
        grid.addWidget(btn_left, 1, 4)
        grid.addWidget(btn_right, 1, 5)

        layout.addWidget(deck_frame, 2)

        # 2. Throttle, centring and the flood current
        ops_pane = QVBoxLayout()

        scale_bar = QHBoxLayout()
        scale_bar.addWidget(QLabel("Throttle Scale:"))
        for p in (0.25, 0.50, 0.75, 1.00):
            b = QPushButton(f"{int(p*100)}%")
            b.setFixedWidth(42)
            b.clicked.connect(lambda _, val=p: self.set_cruise_speed(val))
            scale_bar.addWidget(b)
        ops_pane.addLayout(scale_bar)

        self.btn_center = QPushButton("Toggle Lane Centring [C]")
        self.btn_center.clicked.connect(self.on_toggle_autocenter)
        ops_pane.addWidget(self.btn_center)

        flood_bar = QHBoxLayout()
        self.btn_flood_calm = QPushButton("Current off")
        self.btn_flood_calm.clicked.connect(lambda: self.bridge.trigger_flood(False))
        self.btn_flood_active = QPushButton("Flood current on")
        self.btn_flood_active.setStyleSheet("background-color: #9c6b24; color: #ffffff; font-weight: 600;")
        self.btn_flood_active.clicked.connect(lambda: self.bridge.trigger_flood(True))
        flood_bar.addWidget(self.btn_flood_calm)
        flood_bar.addWidget(self.btn_flood_active)
        ops_pane.addLayout(flood_bar)

        self.lbl_flood_status = QLabel("Water: CALM")
        self.lbl_flood_status.setStyleSheet("color: #7f848e; font-size: 10px; font-family: Monospace;")
        ops_pane.addWidget(self.lbl_flood_status)

        layout.addLayout(ops_pane, 1)

        # 3. 8-Thruster Power Output Matrix
        t_box = QFrame()
        t_box.setStyleSheet("background-color: #121417; border: 1px solid #282c34; border-radius: 3px; padding: 4px;")
        t_box_layout = QVBoxLayout(t_box)
        t_box_layout.addWidget(QLabel("Thruster Load Array (Newtons):"))

        self.thruster_bars = []
        grid_t = QGridLayout()
        for i in range(8):
            lbl_name = QLabel(f"T{i+1}")
            lbl_name.setStyleSheet("font-family: Monospace; font-size: 9px; color: #5c6370;")
            bar = QProgressBar()
            bar.setRange(-30, 30)
            bar.setValue(0)
            bar.setFormat("%v N")
            bar.setFixedHeight(12)
            grid_t.addWidget(lbl_name, i // 2, (i % 2) * 2)
            grid_t.addWidget(bar, i // 2, (i % 2) * 2 + 1)
            self.thruster_bars.append(bar)

        t_box_layout.addLayout(grid_t)
        layout.addWidget(t_box, 1)

        return box

    def connect_signals(self):
        self.bridge.sig_image.connect(self.update_video_canvas)
        self.bridge.sig_telemetry.connect(self.update_telemetry)
        self.bridge.sig_thrusters.connect(self.update_thrusters)
        self.bridge.sig_status.connect(self.update_status)
        self.bridge.sig_detections.connect(self.update_detections)

    # Keyboard Listener (Full hotkey steering inside console)
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

    def update_video_canvas(self, qimg: QImage):
        scaled = qimg.scaled(
            self.lbl_video.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.lbl_video.setPixmap(QPixmap.fromImage(scaled))
        now = time.time()
        if self.last_frame_t is not None and now > self.last_frame_t:
            self.fps = 0.9 * self.fps + 0.1 / (now - self.last_frame_t)
            self.lbl_fps.setText(f"FPS: {self.fps:.1f}")
        self.last_frame_t = now
        self.last_frame = qimg

    def update_telemetry(self, t: dict):
        self.gauge_depth.update_depth(t.get("depth", 0.0))
        self.gauge_attitude.update_attitude(t.get("roll", 0.0), t.get("pitch", 0.0))
        self.map_river.update_pose(t.get("x", 0.0), t.get("y", 0.0), t.get("yaw", 0.0))

        self.lbl_val_x.setText(f"X: {t.get('x', 0.0):+.2f} m")
        self.lbl_val_y.setText(f"Y: {t.get('y', 0.0):+.2f} m")
        self.lbl_val_z.setText(f"Z: {t.get('z', 0.0):+.2f} m")
        self.lbl_val_yaw.setText(f"HDG: {t.get('yaw', 0.0):03.0f}°")

        spd = math.sqrt(t.get("vx", 0.0)**2 + t.get("vy", 0.0)**2)
        self.lbl_val_speed.setText(f"SPD: {spd:.2f} m/s")
        if "offset" in t:
            self.lbl_val_offset.setText(f"LANE: {t['offset']:+.2f} m")

    def update_thrusters(self, vals: list):
        for i, bar in enumerate(self.thruster_bars):
            if i < len(vals):
                val = int(round(vals[i]))
                bar.setValue(val)
                if abs(val) > 24:
                    bar.setStyleSheet("QProgressBar::chunk { background-color: #cd3d3d; }")
                elif abs(val) > 15:
                    bar.setStyleSheet("QProgressBar::chunk { background-color: #b58900; }")
                else:
                    bar.setStyleSheet("QProgressBar::chunk { background-color: #2d8a4e; }")

    def update_status(self, s: dict):
        online = s.get("comms_online", False)
        if online:
            self.lbl_comms.setText("LINK: UP")
            self.lbl_comms.setStyleSheet("color: #98c379; font-family: Monospace; font-weight: 600;")
        else:
            self.lbl_comms.setText("LINK: TIMEOUT / NO SIGNAL")
            self.lbl_comms.setStyleSheet("color: #e06c75; font-family: Monospace; font-weight: 600;")

        state_txt = s.get("safety_state", "UNKNOWN")
        self.lbl_safety.setText(f"SUPERVISOR: {state_txt.upper()}")
        if "ARMED" in state_txt:
            self.lbl_safety.setStyleSheet("background-color: #238636; color: #ffffff; padding: 3px 8px; border-radius: 3px; font-weight: 600; font-size: 10px;")
        else:
            self.lbl_safety.setStyleSheet("background-color: #b58900; color: #ffffff; padding: 3px 8px; border-radius: 3px; font-weight: 600; font-size: 10px;")

        c_on = s.get("auto_center", False)
        self.lbl_autocenter.setText(f"LANE CENTRING: {'ACTIVE' if c_on else 'STANDBY'}")
        self.lbl_autocenter.setStyleSheet(f"color: {'#61afef' if c_on else '#7f848e'}; font-family: Monospace; font-size: 10px; border: 1px solid {'#2b5b84' if c_on else '#3b4048'}; border-radius: 2px; padding: 2px 6px;")

        self.lbl_flood_status.setText(f"Water: {s.get('flood_status', 'CALM')}")

    def update_detections(self, detections: list):
        self.map_river.update_targets(detections)
        now_str = datetime.now().strftime("%H:%M:%S")

        for d in detections:
            cls_name = d.get("class", "object")
            conf = d.get("confidence")
            conf_txt = "--" if conf is None else f"{int(conf * 100)}%"
            rng = d.get("range_m", None)
            xyz = d.get("spatial_xyz", [0.0, 0.0, 0.0])

            if cls_name in self.detected_items_map:
                row_idx = self.detected_items_map[cls_name]["row"]
                self.table_det.item(row_idx, 0).setText(now_str)
                self.table_det.item(row_idx, 2).setText(conf_txt)
                if rng:
                    self.table_det.item(row_idx, 3).setText(f"{rng:.2f} m")
                self.table_det.item(row_idx, 4).setText(f"[{xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}]")
                self.detected_items_map[cls_name]["data"] = d
            else:
                row_idx = self.table_det.rowCount()
                self.table_det.insertRow(row_idx)
                self.table_det.setItem(row_idx, 0, QTableWidgetItem(now_str))
                self.table_det.setItem(row_idx, 1, QTableWidgetItem(cls_name.upper()))
                self.table_det.setItem(row_idx, 2, QTableWidgetItem(conf_txt))
                self.table_det.setItem(row_idx, 3, QTableWidgetItem(f"{rng:.2f} m" if rng else "--"))
                self.table_det.setItem(row_idx, 4, QTableWidgetItem(f"[{xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}]"))
                self.table_det.setItem(row_idx, 5, QTableWidgetItem("camera"))

                self.detected_items_map[cls_name] = {
                    "row": row_idx,
                    "first_seen": now_str,
                    "data": d,
                }

        self.lbl_det_count.setText(f"Classified Contacts: {len(self.detected_items_map)}")
        self.auto_save_detections()

    def auto_save_detections(self):
        try:
            records = []
            for cls_name, info in self.detected_items_map.items():
                records.append({
                    "classification": cls_name,
                    "first_detected": info["first_seen"],
                    "telemetry": info["data"],
                })
            with open(self.auto_save_path, "w") as f:
                json.dump(records, f, indent=2)
        except Exception:
            pass

    def update_clock(self):
        elapsed = datetime.now() - self.start_time
        tot_sec = int(elapsed.total_seconds())
        hh = tot_sec // 3600
        mm = (tot_sec % 3600) // 60
        ss = tot_sec % 60
        self.lbl_clock.setText(f"MISSION TIME: {hh:02d}:{mm:02d}:{ss:02d}")

    def on_stream_changed(self, idx):
        streams = ["detections", "enhanced", "turbid", "split", "raw"]
        if idx < len(streams):
            self.bridge.set_stream(streams[idx])

    def on_snapshot_clicked(self):
        if hasattr(self, "last_frame") and self.last_frame:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fname = os.path.join(self.snapshot_dir, f"frame_{ts}.png")
            self.last_frame.save(fname, "PNG")
            QMessageBox.information(self, "Frame Captured", f"Inspection image archived to:\n{fname}")

    def on_teleop_cmd(self, surge=0.0, sway=0.0, heave=0.0, yaw=0.0):
        self.bridge.send_motion(surge, sway, heave, yaw)

    def on_all_stop(self):
        self.bridge.stop_all()

    def on_emergency_stop(self):
        self.bridge.trigger_kill()
        QMessageBox.warning(self, "EMERGENCY DISARM", "Emergency Disarm Signal Transmitted. All thrusters gated to 0.0 N.")

    def on_toggle_autocenter(self):
        val = self.bridge.toggle_auto_center()
        self.lbl_autocenter.setText(f"LANE CENTRING: {'ACTIVE' if val else 'STANDBY'}")

    def set_cruise_speed(self, val):
        self.cruise_speed = val

    def export_csv(self):
        fname, _ = QFileDialog.getSaveFileName(self, "Export Contact Log to CSV", os.path.expanduser("~/subsea_contacts.csv"), "CSV Files (*.csv)")
        if not fname:
            return
        with open(fname, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Timestamp (UTC)", "Classification", "Confidence", "Range", "Spatial XYZ", "Operator Notes"])
            for r in range(self.table_det.rowCount()):
                row_vals = [self.table_det.item(r, c).text() if self.table_det.item(r, c) else "" for c in range(6)]
                writer.writerow(row_vals)
        QMessageBox.information(self, "Export Successful", f"Saved contact log to:\n{fname}")

    def export_json(self):
        fname, _ = QFileDialog.getSaveFileName(self, "Export Telemetry to JSON", os.path.expanduser("~/subsea_telemetry.json"), "JSON Files (*.json)")
        if not fname:
            return
        records = []
        for r in range(self.table_det.rowCount()):
            records.append({
                "timestamp": self.table_det.item(r, 0).text(),
                "classification": self.table_det.item(r, 1).text(),
                "confidence": self.table_det.item(r, 2).text(),
                "range": self.table_det.item(r, 3).text(),
                "spatial_xyz": self.table_det.item(r, 4).text(),
                "notes": self.table_det.item(r, 5).text(),
            })
        with open(fname, "w") as f:
            json.dump(records, f, indent=2)
        QMessageBox.information(self, "Export Successful", f"Saved JSON telemetry to:\n{fname}")

    def clear_detection_log(self):
        self.table_det.setRowCount(0)
        self.detected_items_map.clear()
        self.lbl_det_count.setText("Classified Contacts: 0")


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
