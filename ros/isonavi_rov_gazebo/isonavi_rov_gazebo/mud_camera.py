"""Synthetic river mud on the Gazebo camera, for the operator view.

Gazebo's fog has no forward scattering and no drifting silt, so this blurs the
frame, adds a brown veil and draws drifting specks. It runs in the simulation
only: the flight vision takes the camera directly.

Subscribes camera/image_raw and flood_active; publishes camera/image_raw_turbid.
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# Specks per layer: count, radius range, drift speed range in px/s, grey range,
# wobble speed and amplitude, blue and green share of the grey.
LAYERS = (
    (180, (1, 3), (25.0, 60.0), (20, 50), np.sin, 2.0, 3.5, 0.70, 0.90),
    (100, (2, 5), (15.0, 40.0), (24, 58), np.cos, 1.5, 5.0, 0.65, 0.85),
    (40, (4, 7), (8.0, 22.0), (18, 48), np.sin, 1.0, 6.5, 0.60, 0.80),
)


def apply_mud(raw_bgr, flood=False, t=0.0):
    h, w = raw_bgr.shape[:2]
    ksize, sigma = (15, 4.2) if flood else (11, 2.8)
    direct = cv2.GaussianBlur(raw_bgr, (ksize, ksize), sigmaX=sigma)
    halo = cv2.GaussianBlur(raw_bgr, (31, 31), sigmaX=8.5)
    blurred = cv2.addWeighted(direct, 0.60, halo, 0.40, 0)

    veil = np.full_like(raw_bgr, (32, 58, 82), dtype=np.uint8)
    alpha = 0.70 if flood else 0.46
    out = cv2.addWeighted(blurred, 1.0 - alpha, veil, alpha, 0)

    # The same seed every frame, so each speck keeps its place and only drifts.
    rng = np.random.RandomState(99)
    for n, rad, spd, grey, wob, wob_w, wob_a, kb, kg in LAYERS:
        x0, y0 = rng.uniform(0, w, n), rng.uniform(0, h, n)
        speed = rng.uniform(*spd, n)
        radius, shade = rng.randint(*rad, n), rng.randint(*grey, n)
        xs = ((x0 + t * speed) % w).astype(np.int32)
        ys = np.clip(y0 + wob(t * wob_w + x0) * wob_a, 0, h - 1).astype(np.int32)
        for i in range(n):
            c = int(shade[i])
            cv2.circle(out, (int(xs[i]), int(ys[i])), int(radius[i]), (int(c * kb), int(c * kg), c), -1)
    return out


class MudCameraNode(Node):
    def __init__(self):
        super().__init__("mud_camera")
        self.bridge = CvBridge()
        self.flood = False
        self.t0 = time.time()
        self.pub = self.create_publisher(Image, "camera/image_raw_turbid", 10)
        self.create_subscription(Bool, "flood_active", self.on_flood, 10)
        self.create_subscription(Image, "camera/image_raw", self.on_image, 2)

    def on_flood(self, m):
        self.flood = bool(m.data)

    def on_image(self, m):
        bgr = self.bridge.imgmsg_to_cv2(m, desired_encoding="bgr8")
        out = self.bridge.cv2_to_imgmsg(apply_mud(bgr, self.flood, time.time() - self.t0), encoding="bgr8")
        out.header = m.header
        self.pub.publish(out)


def main():
    rclpy.init()
    node = MudCameraNode()
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
