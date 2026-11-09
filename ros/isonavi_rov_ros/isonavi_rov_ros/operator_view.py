"""Enhanced video and overlays for the pilot. Nothing here feeds the controller.

The dehaze finds targets further out than the raw frame does, but it changes
the brightness the range estimate depends on, so the flight vision (vision_node)
stays on the raw camera. Detections shown here come from three places:

  targets from vision_node;
  contours of tall piers and wide pipes on the dehazed frame, which give a
  range from the assumed size of the object and have no confidence score;
  a YOLO model, off by default. The stock COCO weights have no pipe or valve
  classes, so pass a trained model: -p enable_yolo:=true -p model_path:=best.pt

Subscribes image_topic (camera/image_raw), targets and flood_active.
Publishes camera/image_enhanced, camera/image_detections, camera/detections.
"""

import json

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

HFOV_DEG = 80.0
TARGET_WIDTH = 160      # vision_node reports targets in pixels of this width
TARGET_RADIUS_M = 0.12

# Assumed size in metres and the box axis it is compared with, for a range.
KNOWN_SIZES = {
    "bridge_pier": (2.4, "width"),
    "pipeline": (0.6, "height"),
    "pipe": (0.6, "height"),
    "buoy": (0.24, "diag"),
    "boat": (4.0, "width"),
    "person": (1.7, "height"),
    "default": (1.0, "max"),
}


def dehaze(img_bgr, flood=False):
    """Dark channel haze removal, then CIELAB stretch, CLAHE, colour balance and sharpening."""
    denoised = cv2.bilateralFilter(img_bgr, d=7, sigmaColor=35, sigmaSpace=35)

    # Veiling light from the brightest part of the blurred dark channel.
    img_f = denoised.astype(np.float32)
    dark = cv2.GaussianBlur(np.min(img_f, axis=2), (31, 31), 0)
    mask = dark >= np.percentile(dark, 95)
    veil = np.median(img_f[mask], axis=0) if np.any(mask) else np.array([35.0, 60.0, 80.0])
    veil = np.clip(veil, 15.0, 240.0)

    # J = (I - A) / t + A
    omega = 0.82 if flood else 0.72
    trans = cv2.GaussianBlur(1.0 - omega * np.min(img_f / veil, axis=2), (25, 25), 0)
    trans = np.clip(trans, 0.18, 1.0)[:, :, np.newaxis]
    restored = np.clip((img_f - veil) / trans + veil, 0.0, 255.0).astype(np.uint8)

    l, a, b = cv2.split(cv2.cvtColor(restored, cv2.COLOR_BGR2LAB))
    lo, hi = np.percentile(l, (1.0, 99.0))
    if hi > lo + 5:
        l = np.clip((l.astype(np.float32) - lo) / (hi - lo) * 255.0, 0, 255).astype(np.uint8)
    l = cv2.createCLAHE(clipLimit=2.2 if flood else 1.8, tileGridSize=(8, 8)).apply(l)

    k = 0.65 if flood else 0.50
    a = np.clip(a.astype(np.float32) - k * (float(np.mean(a)) - 128.0), 0, 255).astype(np.uint8)
    b = np.clip(b.astype(np.float32) - k * (float(np.mean(b)) - 128.0), 0, 255).astype(np.uint8)
    out = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
    return cv2.addWeighted(out, 1.35, cv2.GaussianBlur(out, (0, 0), sigmaX=1.6), -0.35, 0)


def box_range(bbox, name, f_px, cx, cy):
    """Camera frame position (x right, y down, z forward) from the box and an assumed size."""
    x1, y1, x2, y2 = bbox
    w_px, h_px = max(1.0, float(x2 - x1)), max(1.0, float(y2 - y1))
    size, axis = KNOWN_SIZES.get(name.lower(), KNOWN_SIZES["default"])
    if axis == "height":
        px = h_px
    elif axis == "width":
        px = w_px
    elif axis == "diag":
        px = np.hypot(w_px, h_px) / np.sqrt(2.0)
    else:
        px = max(w_px, h_px)
    z = float(np.clip(f_px * size / px, 0.2, 35.0))
    return (round(float(((x1 + x2) / 2.0 - cx) * z / f_px), 2),
            round(float(((y1 + y2) / 2.0 - cy) * z / f_px), 2), round(z, 2))


def find_structures(bgr, f_px, cx, cy):
    """Tall piers and wide pipes as dark contours. Thresholds are for a 640 x 480 frame."""
    grey = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    mask = cv2.adaptiveThreshold(grey, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 25, 4)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for c in contours:
        if cv2.contourArea(c) < 800:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if h > 90 and w > 25 and h / w >= 1.6 and y < 150:
            name = "bridge_pier"
        elif w > 110 and h > 18 and w / h >= 2.5:
            name = "pipeline"
        else:
            continue
        box = [x, y, x + w, y + h]
        xyz = box_range(box, name, f_px, cx, cy)
        found.append({"class": name, "confidence": None, "bbox": box, "range_m": xyz[2], "spatial_xyz": list(xyz)})
    return found


def draw_box(img, bbox, label, color, dist_m=None):
    x1, y1, x2, y2 = [int(v) for v in bbox]
    n = min(16, max(6, (x2 - x1) // 4, (y2 - y1) // 4))
    for (ax, ay, bx, by) in ((x1, y1, n, 0), (x1, y1, 0, n), (x2, y1, -n, 0), (x2, y1, 0, n),
                             (x1, y2, n, 0), (x1, y2, 0, -n), (x2, y2, -n, 0), (x2, y2, 0, -n)):
        cv2.line(img, (ax, ay), (ax + bx, ay + by), color, 2)
    tag = f"{label} [{dist_m:.1f}m]" if dist_m is not None else label
    (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
    ty = max(th + 6, y1 - 4)
    cv2.rectangle(img, (x1, ty - th - 4), (x1 + tw + 6, ty + 2), (18, 18, 18), -1)
    cv2.putText(img, tag, (x1 + 3, ty - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


class OperatorViewNode(Node):
    COLOURS = {"red": (0, 0, 255), "green": (0, 255, 0), "yellow": (0, 255, 255), "blue": (255, 120, 0)}

    def __init__(self):
        super().__init__("operator_view")
        topic = self.declare_parameter("image_topic", "camera/image_raw").value
        model_path = self.declare_parameter("model_path", "yolov8n.pt").value
        self.conf = self.declare_parameter("conf_threshold", 0.25).value
        self.model = None
        if self.declare_parameter("enable_yolo", False).value:
            try:
                from ultralytics import YOLO
                self.model = YOLO(model_path)
            except Exception as e:      # the view still works without it
                self.get_logger().warn(f"no YOLO model ({e})")
        self.bridge = CvBridge()
        self.flood = False
        self.targets = []
        self.pub_enh = self.create_publisher(Image, "camera/image_enhanced", 10)
        self.pub_det = self.create_publisher(Image, "camera/image_detections", 10)
        self.pub_json = self.create_publisher(String, "camera/detections", 10)
        self.create_subscription(Bool, "flood_active", self.on_flood, 10)
        self.create_subscription(String, "targets", self.on_targets, 10)
        self.create_subscription(Image, topic, self.on_image, 2)

    def on_flood(self, m):
        self.flood = bool(m.data)

    def on_targets(self, m):
        self.targets = json.loads(m.data)

    def on_image(self, m):
        bgr = self.bridge.imgmsg_to_cv2(m, desired_encoding="bgr8")
        h, w = bgr.shape[:2]
        f_px = (w / 2.0) / np.tan(np.radians(HFOV_DEG) / 2.0)
        cx, cy = w / 2.0, h / 2.0

        enhanced = dehaze(bgr, self.flood)
        out = self.bridge.cv2_to_imgmsg(enhanced, encoding="bgr8")
        out.header = m.header
        self.pub_enh.publish(out)

        shown = enhanced.copy()
        dets = []
        scale = w / TARGET_WIDTH
        for t in self.targets:
            u, v, rng = t["u"] * scale, t["v"] * scale, float(t["range_m"])
            r = max(6, int(f_px * TARGET_RADIUS_M / rng))
            box = [int(u) - r, int(v) - r, int(u) + r, int(v) + r]
            dets.append({"class": f"balloon_{t['colour']}", "confidence": None, "bbox": box, "range_m": round(rng, 2),
                         "spatial_xyz": [round((u - cx) * rng / f_px, 2), round((v - cy) * rng / f_px, 2), round(rng, 2)]})
            draw_box(shown, box, t["colour"].upper(), self.COLOURS.get(t["colour"], (255, 255, 255)), rng)

        if self.model is not None:
            for b in self.model(enhanced, conf=self.conf, verbose=False)[0].boxes:
                name = self.model.names.get(int(b.cls[0].item()), "object")
                score = float(b.conf[0].item())
                box = [round(x, 1) for x in b.xyxy[0].tolist()]
                xyz = box_range(box, name, f_px, cx, cy)
                dets.append({"class": name, "confidence": round(score, 3), "bbox": box,
                             "range_m": xyz[2], "spatial_xyz": list(xyz)})
                draw_box(shown, box, f"{name.upper()} {int(score * 100)}%", (0, 255, 120), xyz[2])

        for s in find_structures(enhanced, f_px, cx, cy):
            x1, y1 = s["bbox"][:2]
            if not any(abs(d["bbox"][0] - x1) < 40 and abs(d["bbox"][1] - y1) < 40 for d in dets):
                dets.append(s)
                draw_box(shown, s["bbox"], s["class"].replace("_", " ").upper(), (0, 215, 255), s["range_m"])

        out = self.bridge.cv2_to_imgmsg(shown, encoding="bgr8")
        out.header = m.header
        self.pub_det.publish(out)
        self.pub_json.publish(String(data=json.dumps(dets)))


def main():
    rclpy.init()
    node = OperatorViewNode()
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
