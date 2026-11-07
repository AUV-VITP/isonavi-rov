"""Underwater Optical Enhancement and Spatial AI Detection Node.

Subscribes:
  /camera/image_raw           (sensor_msgs/msg/Image)
  /flood_active               (std_msgs/msg/Bool)

Publishes:
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


def fast_pyramidal_retinex(img_bgr: np.ndarray, flood_mode: bool = False) -> np.ndarray:
    """Enhance underwater imagery using Pyramidal Retinex & CIELAB balancing.

    1. Pyramidal Multi-Scale Retinex (MSR) Illumination Decomposition:
       Separates raw frame S into illumination L and surface reflectance R: S = L * R.
       Downsamples 4x to rapidly compute low-frequency Gaussian illumination (sigma=15, sigma=35).
       Reflectance R = S / (L + eps) neutralizes severe attenuation, light falloff, and vignetting.
       Dynamic range is normalized to preserve midtones and details without shadow clipping.
    2. CIELAB Neutral-Point Chromatic Balancing (Anti-Color-Crush):
       Centers chromatic channels a* and b* towards neutral (128.0) using an adaptive damping factor.
       Completely eliminates the unnatural black-and-blue artifact, restoring natural hues.
    3. Luminance Adaptive CLAHE:
       Applies CLAHE strictly on the L* channel (clipLimit 2.0 - 2.5, tileGrid 8x8) to maximize
       underwater contrast without color distorting or blowing highlights.
    4. Silt Denoising & Edge Sharpening (Bilateral + Unsharp Masking):
       Edge-preserving bilateral filter strips suspended sediment specks ('marine snow')
       while high-boost unsharp masking accentuates structural outlines (piers, pipes, buoys).
    """
    h, w = img_bgr.shape[:2]
    img_f = img_bgr.astype(np.float32) + 1.0

    # 1. Multi-scale illumination decomposition (Pyramidal Retinex)
    scale = 4
    small = cv2.resize(img_f, (max(1, w // scale), max(1, h // scale)), interpolation=cv2.INTER_AREA)
    blur1 = cv2.GaussianBlur(small, (0, 0), sigmaX=15.0)
    blur2 = cv2.GaussianBlur(small, (0, 0), sigmaX=35.0)
    illum_small = 0.5 * (blur1 + blur2)
    illum = cv2.resize(illum_small, (w, h), interpolation=cv2.INTER_LINEAR)

    # Reflectance R = S / L
    reflectance = img_f / illum

    # Normalization without crushing shadows: map [0.4, 1.8] to [0, 255]
    norm_r = np.clip((reflectance - 0.4) / (1.8 - 0.4) * 255.0, 0, 255).astype(np.uint8)

    # 2. CIELAB Neutral-Point Chromatic Balancing
    lab = cv2.cvtColor(norm_r, cv2.COLOR_BGR2LAB)
    l_ch, a_ch, b_ch = cv2.split(lab)

    clip_lim = 2.5 if flood_mode else 2.0
    clahe = cv2.createCLAHE(clipLimit=clip_lim, tileGridSize=(8, 8))
    l_eq = clahe.apply(l_ch)

    mean_a = float(np.mean(a_ch))
    mean_b = float(np.mean(b_ch))
    alpha = 0.6 if flood_mode else 0.45
    a_bal = np.clip(a_ch.astype(np.float32) - alpha * (mean_a - 128.0), 0, 255).astype(np.uint8)
    b_bal = np.clip(b_ch.astype(np.float32) - alpha * (mean_b - 128.0), 0, 255).astype(np.uint8)

    lab_out = cv2.merge([l_eq, a_bal, b_bal])
    balanced_bgr = cv2.cvtColor(lab_out, cv2.COLOR_LAB2BGR)

    # 3. Bilateral filter (suspended silt rejection) + Unsharp Masking
    denoised = cv2.bilateralFilter(balanced_bgr, d=5, sigmaColor=25, sigmaSpace=25)
    blurred = cv2.GaussianBlur(denoised, (0, 0), sigmaX=1.5)
    sharpened = cv2.addWeighted(denoised, 1.35, blurred, -0.35, 0)
    return sharpened


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
        self.pub_enhanced = self.create_publisher(Image, "camera/image_enhanced", 10)
        self.pub_detections = self.create_publisher(Image, "camera/image_detections", 10)
        self.pub_det_json = self.create_publisher(String, "camera/detections", 10)
        self.pub_targets = self.create_publisher(String, "targets", 10)
        self.pub_offset = self.create_publisher(Float64, "corridor_offset", 10)

        # Subscription to camera feed and flood alerts
        self.flood_active = False
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

        # 1. Apply Pyramidal Retinex Enhancement
        if self.enable_enhancement:
            enhanced_bgr = fast_pyramidal_retinex(raw_bgr, flood_mode=self.flood_active)
        else:
            enhanced_bgr = raw_bgr.copy()

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
