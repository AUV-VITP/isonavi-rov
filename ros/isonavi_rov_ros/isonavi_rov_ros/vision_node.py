"""Camera frames to target detections, environmental inspections, and corridor offset.

targets           JSON list of detections with class, condition, bbox/centroid and range
corridor_offset   the vehicle's offset from the corridor centreline in metres,
                  from the wall brightness ratio; meaningful only inside a
                  corridor, which is why the pilot engages it by hand
""" 

import json
import os
import sys

# Ensure simulation package is discoverable from this path
ROOT_SIM = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../simulation"))
if ROOT_SIM not in sys.path:
    sys.path.insert(0, ROOT_SIM)

import numpy as np

# ROS 2 imports with host-environment fallback for standalone testing and IDE linting
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float64, String
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

    class Node:
        def __init__(self, name):
            self._name = name

        def declare_parameter(self, name, default_val):
            class _Param:
                def __init__(self, val):
                    self.value = val
            return _Param(default_val)

        def create_subscription(self, *args, **kwargs):
            return None

        def create_publisher(self, *args, **kwargs):
            class _Pub:
                def publish(self, msg):
                    pass
            return _Pub()

        def get_logger(self):
            class _Logger:
                def info(self, msg):
                    print(f"[{self._name}] {msg}")
            _l = _Logger()
            _l._name = self._name
            return _l

    class _MsgStub:
        def __init__(self):
            self.data = None
            self.encoding = "rgb8"
            self.height = 480
            self.width = 640

    Image = Float64 = String = _MsgStub

from isonavi.vision import CourseEstimator, TargetFinder, load_calibration

try:
    import cv2
    import onnxruntime as ort
    HAS_ONNX = True
except ImportError:
    HAS_ONNX = False

DETECTOR_CLASSES = ("fish", "pipe", "debris", "structure")
CONDITION_CLASSES = ("normal", "damaged", "unknown")


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision")
        b = self.declare_parameter("scattering", 0.4).value
        w = self.declare_parameter("width", 160).value
        self.corridor_w = self.declare_parameter("corridor_width", 1.0).value
        self.use_onnx = self.declare_parameter("use_onnx", True).value and HAS_ONNX
        self.conf_thresh = self.declare_parameter("conf_threshold", 0.35).value

        # Baseline heuristic course & target estimators
        self.finder = TargetFinder(width=w, scattering=b)
        self.course = CourseEstimator(width=w, height=int(w * 3 / 4), scattering=b,
                                      calib=load_calibration(b))

        # ONNX sessions for YOLO11n object detector and MobileNetV3 defect classifier
        self.detector_sess = None
        self.classifier_sess = None
        if self.use_onnx:
            root = os.environ.get("ISONAVI_ROOT", os.path.abspath(os.path.join(os.path.dirname(__file__), "../../..")))
            det_path = os.path.join(root, "cv", "results", "Detector", "best.onnx")
            cls_path = os.path.join(root, "cv", "results", "Classifier", "aegis_condition_classifier.onnx")
            if os.path.exists(det_path):
                self.detector_sess = ort.InferenceSession(det_path, providers=["CPUExecutionProvider"])
                self.get_logger().info(f"Loaded YOLO11n detector from {det_path}")
            if os.path.exists(cls_path):
                self.classifier_sess = ort.InferenceSession(cls_path, providers=["CPUExecutionProvider"])
                self.get_logger().info(f"Loaded condition classifier from {cls_path}")

        self.create_subscription(Image, "camera/image_raw", self.on_image, 2)
        self.pub = self.create_publisher(String, "targets", 10)
        self.off_pub = self.create_publisher(Float64, "corridor_offset", 10)

    def run_onnx_pipeline(self, img_rgb):
        """Run YOLO11n detection (384x384) and classify condition on candidate crops."""
        h_orig, w_orig = img_rgb.shape[:2]
        resized = cv2.resize(img_rgb, (384, 384))
        inp = resized.transpose(2, 0, 1).astype(np.float32) / 255.0
        inp = np.expand_dims(inp, axis=0)

        outputs = self.detector_sess.run(["output0"], {"images": inp})[0][0]  # [8, 3024]
        # Channels: xc, yc, w, h, score_fish, score_pipe, score_debris, score_structure
        boxes = outputs[:4, :].T  # [3024, 4]
        scores = outputs[4:, :].T  # [3024, 4]

        class_ids = np.argmax(scores, axis=1)
        confidences = np.max(scores, axis=1)
        mask = confidences >= self.conf_thresh

        valid_boxes = boxes[mask]
        valid_cls = class_ids[mask]
        valid_confs = confidences[mask]

        detections = []
        scale_x, scale_y = w_orig / 384.0, h_orig / 384.0

        for box, cid, conf in zip(valid_boxes, valid_cls, valid_confs):
            xc, yc, bw, bh = box
            u, v = float(xc * scale_x), float(yc * scale_y)
            w_px, h_px = float(bw * scale_x), float(bh * scale_y)
            label = DETECTOR_CLASSES[cid]

            condition = "unclassified"
            # If structure or pipe is detected, run defect condition classifier on crop
            if label in ("pipe", "structure") and self.classifier_sess is not None:
                x1 = max(0, int(u - w_px / 2))
                y1 = max(0, int(v - h_px / 2))
                x2 = min(w_orig, int(u + w_px / 2))
                y2 = min(h_orig, int(v + h_px / 2))
                if x2 > x1 + 5 and y2 > y1 + 5:
                    crop = img_rgb[y1:y2, x1:x2]
                    crop_res = cv2.resize(crop, (224, 224)).transpose(2, 0, 1).astype(np.float32) / 255.0
                    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(3, 1, 1)
                    std = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(3, 1, 1)
                    crop_norm = np.expand_dims((crop_res - mean) / std, axis=0)
                    logits = self.classifier_sess.run(["logits"], {"input": crop_norm})[0][0]
                    condition = CONDITION_CLASSES[int(np.argmax(logits))]

            # Pinhole range approximation from apparent size
            f_px = (w_orig / 2.0) / np.tan(np.radians(40.0))
            nominal_size_m = 0.30 if label == "pipe" else 0.40
            apparent_size_px = max(w_px, h_px, 1.0)
            range_est = float(np.clip(f_px * nominal_size_m / apparent_size_px, 0.3, 10.0))

            detections.append({
                "class": label,
                "confidence": float(round(conf, 3)),
                "condition": condition,
                "u": float(round(u, 1)),
                "v": float(round(v, 1)),
                "w": float(round(w_px, 1)),
                "h": float(round(h_px, 1)),
                "range_m": float(round(range_est, 2)),
            })

        return detections

    def on_image(self, m):
        if m.encoding not in ("rgb8", "bgr8"):
            return
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        if m.encoding == "bgr8":
            img = img[:, :, ::-1]

        # Prioritize ONNX inference when available; fallback to heuristic TargetFinder
        if self.detector_sess is not None:
            results = self.run_onnx_pipeline(img)
        else:
            results = [{"colour": d.colour, "class": "target", "u": d.centroid[0],
                        "v": d.centroid[1], "range_m": d.range_m} for d in self.finder.find(img)]

        out = String()
        out.data = json.dumps(results)
        self.pub.publish(out)

        # Heading is held by the assist, so the offset is taken at zero relative yaw.
        off = Float64()
        off.data = float(self.course.corridor(img, 0.0, 0.0, self.corridor_w))
        self.off_pub.publish(off)


def main():
    if HAS_ROS2:
        rclpy.init()
        rclpy.spin(VisionNode())
    else:
        print("[vision_node] ROS 2 (rclpy) not found in host environment. Running standalone ONNX pipeline test...")
        node = VisionNode()
        test_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        if node.detector_sess is not None:
            import time
            t0 = time.time()
            res = node.run_onnx_pipeline(test_img)
            dt_ms = (time.time() - t0) * 1000.0
            print(f"[vision_node] Self-test PASS: Inference ran in {dt_ms:.1f} ms ({1000.0/max(dt_ms, 1e-3):.1f} FPS), processed {len(res)} candidate objects.")
        else:
            print("[vision_node] Detector model not found.")


if __name__ == "__main__":
    main()
