"""Camera frames to coloured target detections, published as JSON."""

import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from isonavi.vision import TargetFinder


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision")
        b = self.declare_parameter("scattering", 0.4).value
        w = self.declare_parameter("width", 160).value
        self.finder = TargetFinder(width=w, scattering=b)
        self.create_subscription(Image, "camera/image_raw", self.on_image, 2)
        self.pub = self.create_publisher(String, "targets", 10)

    def on_image(self, m):
        if m.encoding not in ("rgb8", "bgr8"):
            return
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        if m.encoding == "bgr8":
            img = img[:, :, ::-1]
        out = String()
        out.data = json.dumps([{"colour": d.colour, "u": d.centroid[0], "v": d.centroid[1],
                                "range_m": d.range_m} for d in self.finder.find(img)])
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(VisionNode())


if __name__ == "__main__":
    main()
