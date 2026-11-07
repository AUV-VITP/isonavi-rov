"""Underwater Optical Enhancement and Spatial AI Detection Node.

Subscribes:
  /camera/image_raw           (sensor_msgs/msg/Image)
  /flood_active               (std_msgs/msg/Bool)

Publishes:
  /camera/image_raw_turbid    (sensor_msgs/msg/Image) - River silt & mud-blurred optical stream with drifting flakes
  /camera/image_enhanced      (sensor_msgs/msg/Image) - Dehazed, color-balanced, sharp frame via Pyramidal Retinex
  /camera/image_detections    (sensor_msgs/msg/Image) - Frame annotated with Tactical HUD, YOLOv8 & Spatial AI
  /camera/detections          (std_msgs/msg/String)   - JSON detection list with bboxes, labels, confidences, 3D XYZ
  /targets                    (std_msgs/msg/String)   - Balloon target locations for autonomy
  /corridor_offset            (std_msgs/msg/Float64)  - Centreline lateral offset for pilot assist
"""

import json
import os
import sys
import time

# Ensure simulation module can be imported regardless of execution context
sim_candidates = [
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../simulation")),
    os.path.expanduser("~/isonavi/isonavi-rov/simulation"),
    os.path.join(os.getcwd(), "simulation"),
    os.path.join(os.getcwd(), "isonavi-rov", "simulation"),
]
for p in sim_candidates:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64, String

from isonavi.vision import CourseEstimator, TargetFinder, load_calibration

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


# Known physical dimensions for river infrastructure and marine objects (meters)
# Inspired by IIT Bombay Spatial AI and RoboSub competition specs
KNOWN_OBJECT_SIZES = {
    "bridge_pier": {"dim": 2.4, "axis": "width"},      # 2.4m diameter concrete pier
    "pier": {"dim": 2.4, "axis": "width"},
    "column": {"dim": 2.4, "axis": "width"},
    "pipeline": {"dim": 0.6, "axis": "height"},        # 0.6m diameter riverbed crossing pipe
    "pipe": {"dim": 0.6, "axis": "height"},
    "balloon_red": {"dim": 0.3, "axis": "diag"},       # 0.3m buoy
    "balloon_green": {"dim": 0.3, "axis": "diag"},
    "balloon_yellow": {"dim": 0.3, "axis": "diag"},
    "balloon_blue": {"dim": 0.3, "axis": "diag"},
    "buoy": {"dim": 0.3, "axis": "diag"},
    "boat": {"dim": 4.0, "axis": "width"},
    "person": {"dim": 1.7, "axis": "height"},
    "bottle": {"dim": 0.3, "axis": "height"},          # COCO proxy for floating buoy/marker
    "sports ball": {"dim": 0.3, "axis": "diag"},
    "default": {"dim": 1.0, "axis": "max"},
}


def apply_mud_turbidity(
    raw_bgr: np.ndarray,
    scattering: float = 0.65,
    flood_mode: bool = False,
    sim_time: float = 0.0,
) -> np.ndarray:
    """Simulate physical river mud forward-scattering blur, suspended micro-silt & veiling luminance.

    Gazebo OGRE2 cameras apply distance-based fog blending, but lack optical Point Spread Function
    (PSF) forward scattering of micro-dirt particles and suspended colloidal sediment.
    This function physically simulates:
    1. Forward-Scattering Modulation Transfer Function (MTF) blur:
       High spatial frequency attenuation through dense colloidal sediment (PSF kernel).
    2. Silt & River Mud Backscatter (Veiling Luminance):
       Brownish-ochre ambient backscatter from sunlight scattering off suspended clay/silt.
    3. Dense Floating Suspended Micro-Dirt Specks & Silt Flakes:
       320 continuous drifting mud specks and organic silt grains in multiple fluid layers.
    """
    h, w = raw_bgr.shape[:2]

    # 1. Optical Forward-Scattering Point Spread Function (PSF) Blur
    ksize = 15 if flood_mode else 11
    sigma = 4.2 if flood_mode else 2.8
    blur_direct = cv2.GaussianBlur(raw_bgr, (ksize, ksize), sigmaX=sigma)
    blur_halo = cv2.GaussianBlur(raw_bgr, (31, 31), sigmaX=8.5)
    optical_blur = cv2.addWeighted(blur_direct, 0.60, blur_halo, 0.40, 0)

    # 2. Muddy Ochre-Tan Veiling Luminance (Sediment Backscatter)
    veil_bgr = np.full_like(raw_bgr, (32, 58, 82), dtype=np.uint8)
    veil_alpha = 0.46 + (0.24 if flood_mode else 0.0)
    turbid = cv2.addWeighted(optical_blur, 1.0 - veil_alpha, veil_bgr, veil_alpha, 0)

    # 3. Dense Suspended Micro-Dirt Specks & Drifting Silt Particles (320 specks across 3 layers)
    np.random.seed(99)
    # Layer 1: 180 fine micro-silt specks (r = 1-2) drifting rapidly with river current
    n1 = 180
    b_x1 = np.random.uniform(0, w, n1)
    b_y1 = np.random.uniform(0, h, n1)
    spd1 = np.random.uniform(25.0, 60.0, n1)
    cur_x1 = ((b_x1 + sim_time * spd1) % w).astype(np.int32)
    cur_y1 = np.clip(b_y1 + np.sin(sim_time * 2.0 + b_x1) * 3.5, 0, h - 1).astype(np.int32)
    r1 = np.random.randint(1, 3, n1)
    for i in range(n1):
        fc = int(np.random.randint(20, 50))
        cv2.circle(turbid, (int(cur_x1[i]), int(cur_y1[i])), int(r1[i]), (int(fc * 0.7), int(fc * 0.9), int(fc)), -1)

    # Layer 2: 100 medium mud flakes (r = 2-4) with eddy tumbling
    n2 = 100
    b_x2 = np.random.uniform(0, w, n2)
    b_y2 = np.random.uniform(0, h, n2)
    spd2 = np.random.uniform(15.0, 40.0, n2)
    cur_x2 = ((b_x2 + sim_time * spd2) % w).astype(np.int32)
    cur_y2 = np.clip(b_y2 + np.cos(sim_time * 1.5 + b_x2) * 5.0, 0, h - 1).astype(np.int32)
    r2 = np.random.randint(2, 5, n2)
    for i in range(n2):
        fc = int(np.random.randint(24, 58))
        cv2.circle(turbid, (int(cur_x2[i]), int(cur_y2[i])), int(r2[i]), (int(fc * 0.65), int(fc * 0.85), int(fc)), -1)

    # Layer 3: 40 large organic mud clumps (r = 4-6)
    n3 = 40
    b_x3 = np.random.uniform(0, w, n3)
    b_y3 = np.random.uniform(0, h, n3)
    spd3 = np.random.uniform(8.0, 22.0, n3)
    cur_x3 = ((b_x3 + sim_time * spd3) % w).astype(np.int32)
    cur_y3 = np.clip(b_y3 + np.sin(sim_time * 1.0 + b_x3) * 6.5, 0, h - 1).astype(np.int32)
    r3 = np.random.randint(4, 7, n3)
    for i in range(n3):
        fc = int(np.random.randint(18, 48))
        cv2.circle(turbid, (int(cur_x3[i]), int(cur_y3[i])), int(r3[i]), (int(fc * 0.6), int(fc * 0.8), int(fc)), -1)

    return turbid


def fast_pyramidal_retinex(img_bgr: np.ndarray, flood_mode: bool = False) -> np.ndarray:
    """Enhance underwater imagery using Dark-Channel Radiance Recovery & CIELAB equalization.

    1. Suspended Silt Rejection: Edge-preserving bilateral filter strips drifting sediment flakes.
    2. Atmospheric / Veiling Backscatter Estimation: Automatically derives veiling light A [B, G, R].
    3. Transmission Radiance Recovery: Inverts the underwater optical transmission equation
       J = (I - A) / t + A, canceling out the mud turbidity veil.
    4. Adaptive Dynamic Range Stretching: Stretches L* channel to full [0, 255] via 1%-99% percentiles.
    5. Neutral-Point Chromatic Balancing: Centers a* and b* to eliminate brownish-yellow casts.
    6. High-Boost Unsharp Masking: Sharpens high-frequency outlines of pipes, buoys, and piers.
    """
    # 1. Bilateral speckle filter to eliminate suspended sediment grains first
    denoised = cv2.bilateralFilter(img_bgr, d=7, sigmaColor=35, sigmaSpace=35)

    # 2. Estimate ambient mud veiling light A (dark channel)
    img_f = denoised.astype(np.float32)
    dark_channel = np.min(img_f, axis=2)
    dark_blur = cv2.GaussianBlur(dark_channel, (31, 31), 0)
    top_val = np.percentile(dark_blur, 95)
    veil_mask = dark_blur >= top_val
    A = np.median(img_f[veil_mask], axis=0) if np.any(veil_mask) else np.array([35.0, 60.0, 80.0])
    A = np.clip(A, 15.0, 240.0)

    # 3. Transmission map t(x) with haze subtraction
    omega = 0.82 if flood_mode else 0.72
    norm_img = img_f / A
    raw_transmission = 1.0 - omega * np.min(norm_img, axis=2)
    transmission = cv2.GaussianBlur(raw_transmission, (25, 25), 0)
    transmission = np.clip(transmission, 0.18, 1.0)[:, :, np.newaxis]

    # Recover scene radiance: J = (I - A) / t + A
    dehazed = (img_f - A) / transmission + A
    dehazed = np.clip(dehazed, 0.0, 255.0).astype(np.uint8)

    # 4. Dynamic Range & Contrast Adaptive Equalization (CIELAB)
    lab = cv2.cvtColor(dehazed, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)

    p1, p99 = np.percentile(l, (1.0, 99.0))
    if p99 > p1 + 5:
        l_stretched = np.clip((l.astype(np.float32) - p1) / (p99 - p1) * 255.0, 0, 255).astype(np.uint8)
    else:
        l_stretched = l

    clahe = cv2.createCLAHE(clipLimit=2.2 if flood_mode else 1.8, tileGridSize=(8, 8))
    l_clahe = clahe.apply(l_stretched)

    # Neutralize muddy yellowish cast (white balance / chromatic centering)
    a_mean = float(np.mean(a))
    b_mean = float(np.mean(b))
    alpha = 0.65 if flood_mode else 0.50
    a_bal = np.clip(a.astype(np.float32) - alpha * (a_mean - 128.0), 0, 255).astype(np.uint8)
    b_bal = np.clip(b.astype(np.float32) - alpha * (b_mean - 128.0), 0, 255).astype(np.uint8)

    restored = cv2.cvtColor(cv2.merge([l_clahe, a_bal, b_bal]), cv2.COLOR_LAB2BGR)

    # 5. High-Boost Unsharp Masking for crisp edge recovery
    blur_sharp = cv2.GaussianBlur(restored, (0, 0), sigmaX=1.6)
    sharp = cv2.addWeighted(restored, 1.35, blur_sharp, -0.35, 0)
    return sharp


def estimate_spatial_pose(bbox, cls_name, f_px=381.36, cx=320.0, cy=240.0):
    """Estimate 3D spatial position (X, Y, Z in meters) using Spatial AI pinhole projection."""
    x1, y1, x2, y2 = bbox
    w_px = max(1.0, float(x2 - x1))
    h_px = max(1.0, float(y2 - y1))
    u = (x1 + x2) / 2.0
    v = (y1 + y2) / 2.0

    matched = KNOWN_OBJECT_SIZES.get(cls_name.lower())
    if matched is None:
        for k, v_spec in KNOWN_OBJECT_SIZES.items():
            if k in cls_name.lower():
                matched = v_spec
                break
    if matched is None:
        matched = KNOWN_OBJECT_SIZES["default"]

    real_dim = matched["dim"]
    axis = matched["axis"]
    if axis == "height":
        z = (f_px * real_dim) / h_px
    elif axis == "width":
        z = (f_px * real_dim) / w_px
    elif axis == "diag":
        diag_px = np.sqrt(w_px * w_px + h_px * h_px)
        z = (f_px * real_dim) / (diag_px / 1.414)
    else:
        dim_px = max(w_px, h_px)
        z = (f_px * real_dim) / dim_px

    z = float(np.clip(z, 0.2, 35.0))
    x = float((u - cx) * z / f_px)
    y = float((v - cy) * z / f_px)
    return round(x, 2), round(y, 2), round(z, 2)


def draw_tactical_hud(img, bbox, label, color=(0, 255, 255), dist_m=None):
    """Draw sleek corner brackets and Tactical HUD labels."""
    x1, y1, x2, y2 = [int(v) for v in bbox]
    l = min(16, max(6, (x2 - x1) // 4, (y2 - y1) // 4))
    t = 2

    # Corner brackets
    cv2.line(img, (x1, y1), (x1 + l, y1), color, t)
    cv2.line(img, (x1, y1), (x1, y1 + l), color, t)
    cv2.line(img, (x2, y1), (x2 - l, y1), color, t)
    cv2.line(img, (x2, y1), (x2, y1 + l), color, t)
    cv2.line(img, (x1, y2), (x1 + l, y2), color, t)
    cv2.line(img, (x1, y2), (x1, y2 - l), color, t)
    cv2.line(img, (x2, y2), (x2 - l, y2), color, t)
    cv2.line(img, (x2, y2), (x2, y2 - l), color, t)

    # Label text with dark semi-transparent backing
    tag = f"{label} [{dist_m:.1f}m]" if dist_m is not None else label
    (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
    tag_y = max(th + 6, y1 - 4)
    cv2.rectangle(img, (x1, tag_y - th - 4), (x1 + tw + 6, tag_y + 2), (18, 18, 18), -1)
    cv2.putText(img, tag, (x1 + 3, tag_y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


def detect_river_infrastructure(enhanced_bgr: np.ndarray, f_px=381.36, cx=320.0, cy=240.0):
    """Classical OpenCV contour detector for vertical bridge piers and crossing pipelines.

    Provides deterministic infrastructure tracking complementary to deep learning.
    """
    gray = cv2.cvtColor(enhanced_bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.adaptiveThreshold(
        blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 25, 4
    )
    cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    infra_dets = []
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 800:
            continue
        x, y, w, h = cv2.boundingRect(c)

        # Bridge pier: tall vertical profile
        if h > 90 and w > 25 and (h / w) >= 1.6 and y < 150:
            x_m, y_m, z_m = estimate_spatial_pose([x, y, x + w, y + h], "bridge_pier", f_px, cx, cy)
            infra_dets.append({
                "class": "bridge_pier",
                "confidence": 0.88,
                "bbox": [x, y, x + w, y + h],
                "range_m": z_m,
                "spatial_xyz": [x_m, y_m, z_m],
            })
        # Pipeline: wide horizontal cylinder profile
        elif w > 110 and h > 18 and (w / h) >= 2.5:
            x_m, y_m, z_m = estimate_spatial_pose([x, y, x + w, y + h], "pipeline", f_px, cx, cy)
            infra_dets.append({
                "class": "pipeline",
                "confidence": 0.85,
                "bbox": [x, y, x + w, y + h],
                "range_m": z_m,
                "spatial_xyz": [x_m, y_m, z_m],
            })
    return infra_dets


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision")

        # Declare parameters
        self.scattering = self.declare_parameter("scattering", 0.4).value
        self.width = self.declare_parameter("width", 640).value
        self.height = self.declare_parameter("height", 480).value
        self.corridor_w = self.declare_parameter("corridor_width", 1.0).value
        self.model_path = self.declare_parameter("model_path", "yolov8n.pt").value
        self.conf_thresh = self.declare_parameter("conf_threshold", 0.25).value
        self.enable_enhancement = self.declare_parameter("enable_enhancement", True).value
        self.enable_yolo = self.declare_parameter("enable_yolo", True).value

        # Initialize physical optical estimators
        self.calib = load_calibration(self.scattering)
        self.init_estimators(self.width, self.height)

        # Camera pinhole intrinsics (80 deg HFOV @ 640x480)
        self.fx = (self.width / 2.0) / np.tan(np.radians(80.0) / 2.0)
        self.fy = self.fx
        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        # Initialize YOLOv8 detector
        self.model = None
        if self.enable_yolo and ULTRALYTICS_AVAILABLE:
            try:
                self.get_logger().info(f"Loading YOLO model from: {self.model_path}")
                self.model = YOLO(self.model_path)
                self.get_logger().info(f"YOLO loaded successfully with {len(self.model.names)} classes.")
            except Exception as e:
                self.get_logger().warn(f"Could not load YOLO model ({e}). Continuing with optical target detector.")
                self.model = None

        self.bridge = CvBridge()

        # Publishers
        self.pub_turbid = self.create_publisher(Image, "camera/image_raw_turbid", 10)
        self.pub_enhanced = self.create_publisher(Image, "camera/image_enhanced", 10)
        self.pub_detections = self.create_publisher(Image, "camera/image_detections", 10)
        self.pub_det_json = self.create_publisher(String, "camera/detections", 10)
        self.pub_targets = self.create_publisher(String, "targets", 10)
        self.pub_offset = self.create_publisher(Float64, "corridor_offset", 10)

        # Subscription to camera feed and flood alerts
        self.flood_active = False
        self.start_time = time.time()
        self.last_frame_time = time.time()
        self.fps = 0.0
        self.sub_flood = self.create_subscription(Bool, "flood_active", self.on_flood_active, 10)
        self.sub_cam = self.create_subscription(Image, "camera/image_raw", self.on_image, 2)
        self.get_logger().info("VisionNode (ISONAVI Optical Enhancement & Spatial AI) initialized.")

    def on_flood_active(self, m: Bool):
        self.flood_active = bool(m.data)

    def init_estimators(self, w: int, h: int):
        self.w, self.h = w, h
        self.finder = TargetFinder(width=w, scattering=self.scattering)
        self.course = CourseEstimator(
            width=w,
            height=h,
            scattering=self.scattering,
            calib=self.calib,
        )

    def on_image(self, m: Image):
        if m.encoding not in ("rgb8", "bgr8"):
            return

        now = time.time()
        dt = now - self.last_frame_time
        if dt > 0:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)
        self.last_frame_time = now

        # Adapt dynamically if image resolution changes
        if m.width != self.w or m.height != self.h:
            self.init_estimators(m.width, m.height)
            self.fx = (m.width / 2.0) / np.tan(np.radians(80.0) / 2.0)
            self.fy = self.fx
            self.cx = m.width / 2.0
            self.cy = m.height / 2.0

        # Parse raw frame
        raw_bgr = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        if m.encoding == "rgb8":
            raw_bgr = cv2.cvtColor(raw_bgr, cv2.COLOR_RGB2BGR)

        # 1. Physical River Mud Turbidity & Suspended Silt Blur Simulation
        turbid_bgr = apply_mud_turbidity(
            raw_bgr,
            scattering=self.scattering,
            flood_mode=self.flood_active,
            sim_time=now - self.start_time,
        )

        # Publish turbid camera stream
        msg_turbid = self.bridge.cv2_to_imgmsg(turbid_bgr, encoding="bgr8")
        msg_turbid.header = m.header
        self.pub_turbid.publish(msg_turbid)

        # 2. Apply Pyramidal Retinex Enhancement (Dehaze mud, remove blur & silt, restore colors)
        if self.enable_enhancement:
            enhanced_bgr = fast_pyramidal_retinex(turbid_bgr, flood_mode=self.flood_active)
        else:
            enhanced_bgr = turbid_bgr.copy()

        # Publish enhanced image stream
        msg_enhanced = self.bridge.cv2_to_imgmsg(enhanced_bgr, encoding="bgr8")
        msg_enhanced.header = m.header
        self.pub_enhanced.publish(msg_enhanced)

        # 2. Extract targets & corridor offset for pilot autonomy
        img_rgb = cv2.cvtColor(enhanced_bgr, cv2.COLOR_BGR2RGB)
        target_detections = self.finder.find(img_rgb)
        targets_json = [
            {
                "colour": d.colour,
                "u": float(d.centroid[0]),
                "v": float(d.centroid[1]),
                "range_m": float(d.range_m),
            }
            for d in target_detections
        ]
        self.pub_targets.publish(String(data=json.dumps(targets_json)))

        # Corridor offset (at zero relative yaw held by assist controller)
        try:
            offset_val = float(self.course.corridor(img_rgb, 0.0, 0.0, self.corridor_w))
            self.pub_offset.publish(Float64(data=offset_val))
        except Exception:
            pass

        # 3. Object Detection & Spatial AI (Tactical AI)
        all_detections = []
        annotated_bgr = enhanced_bgr.copy()

        # (a) YOLO Inference
        if self.model is not None:
            try:
                results = self.model(enhanced_bgr, conf=self.conf_thresh, verbose=False)[0]
                for box in results.boxes:
                    cls_id = int(box.cls[0].item())
                    name = self.model.names.get(cls_id, str(cls_id))
                    conf = float(box.conf[0].item())
                    xyxy = box.xyxy[0].tolist()
                    bbox = [round(x, 1) for x in xyxy]
                    x_m, y_m, z_m = estimate_spatial_pose(bbox, name, self.fx, self.cx, self.cy)

                    all_detections.append({
                        "class": name,
                        "confidence": round(conf, 3),
                        "bbox": bbox,
                        "range_m": z_m,
                        "spatial_xyz": [x_m, y_m, z_m],
                    })

                    # Draw Tactical corner bracket HUD
                    draw_tactical_hud(
                        annotated_bgr,
                        bbox,
                        f"{name.upper()} {int(conf * 100)}%",
                        color=(0, 255, 120),
                        dist_m=z_m,
                    )
            except Exception as e:
                self.get_logger().error(f"YOLO inference error: {e}")

        # (b) Optical Balloon / Buoy Detections
        color_map = {
            "red": (0, 0, 255),
            "green": (0, 255, 0),
            "yellow": (0, 255, 255),
            "blue": (255, 120, 0),
        }
        for d in target_detections:
            u, v = int(d.centroid[0]), int(d.centroid[1])
            r = max(6, int(d.radius_px))
            bbox = [u - r, v - r, u + r, v + r]
            rng = float(d.range_m)
            x_m = round(float((u - self.cx) * rng / self.fx), 2)
            y_m = round(float((v - self.cy) * rng / self.fy), 2)
            conf = round(1.0 - min(1.0, float(d.score)), 3)

            all_detections.append({
                "class": f"balloon_{d.colour}",
                "confidence": conf,
                "bbox": bbox,
                "range_m": round(rng, 2),
                "spatial_xyz": [x_m, y_m, round(rng, 2)],
            })

            circle_color = color_map.get(d.colour, (255, 255, 255))
            draw_tactical_hud(
                annotated_bgr,
                bbox,
                f"BUOY:{d.colour.upper()}",
                color=circle_color,
                dist_m=rng,
            )
            cv2.circle(annotated_bgr, (u, v), 3, (0, 0, 255), -1)

        # (c) River Structural Infrastructure Contour Detection
        infra_dets = detect_river_infrastructure(enhanced_bgr, self.fx, self.cx, self.cy)
        for infra in infra_dets:
            # Avoid duplicating YOLO detection if overlap exists
            x1, y1, x2, y2 = infra["bbox"]
            overlap = any(
                abs(d["bbox"][0] - x1) < 40 and abs(d["bbox"][1] - y1) < 40
                for d in all_detections
            )
            if not overlap:
                all_detections.append(infra)
                draw_tactical_hud(
                    annotated_bgr,
                    infra["bbox"],
                    infra["class"].replace("_", " ").upper(),
                    color=(0, 215, 255),
                    dist_m=infra["range_m"],
                )

        # 4. Tactical Optical HUD Overlay
        # Central crosshair
        ch_len = 10
        cv2.line(annotated_bgr, (int(self.cx) - ch_len, int(self.cy)), (int(self.cx) + ch_len, int(self.cy)), (200, 200, 200), 1)
        cv2.line(annotated_bgr, (int(self.cx), int(self.cy) - ch_len), (int(self.cx), int(self.cy) + ch_len), (200, 200, 200), 1)

        # Header status badge
        mode_str = "FLOOD TURBIDITY ACTIVE" if self.flood_active else "CLEAR WATER"
        badge_text = f"ISONAVI-ROV | {mode_str} | FPS:{self.fps:.1f} | TARGETS:{len(all_detections)}"
        badge_color = (0, 140, 255) if self.flood_active else (0, 255, 120)

        (tw, th), _ = cv2.getTextSize(badge_text, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
        cv2.rectangle(annotated_bgr, (8, 6), (14 + tw, 12 + th + 6), (15, 15, 15), -1)
        cv2.putText(
            annotated_bgr,
            badge_text,
            (11, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            badge_color,
            1,
            cv2.LINE_AA,
        )

        # Publish annotated detection stream & JSON metadata
        msg_annotated = self.bridge.cv2_to_imgmsg(annotated_bgr, encoding="bgr8")
        msg_annotated.header = m.header
        self.pub_detections.publish(msg_annotated)
        self.pub_det_json.publish(String(data=json.dumps(all_detections)))


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
