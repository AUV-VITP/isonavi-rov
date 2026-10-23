"""Camera frames to coloured target detections and the corridor offset.

targets           JSON list of detections with colour, pixel and range
corridor_offset   the vehicle's offset from the corridor centreline in metres,
                  from the wall brightness ratio; meaningful only inside a
                  corridor, which is why the pilot engages it by hand
"""

import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String

from isonavi.vision import CourseEstimator, TargetFinder, load_calibration


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision")
        b = self.declare_parameter("scattering", 0.4).value
        w = self.declare_parameter("width", 160).value
        self.corridor_w = self.declare_parameter("corridor_width", 1.0).value
        self.finder = TargetFinder(width=w, scattering=b)
        self.course = CourseEstimator(width=w, height=int(w * 3 / 4), scattering=b,
                                      calib=load_calibration(b))
        self.create_subscription(Image, "camera/image_raw", self.on_image, 2)
        self.pub = self.create_publisher(String, "targets", 10)
        self.off_pub = self.create_publisher(Float64, "corridor_offset", 10)

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
        # Heading is held by the assist, so the offset is taken at zero relative yaw.
        off = Float64()
        off.data = float(self.course.corridor(img, 0.0, 0.0, self.corridor_w))
        self.off_pub.publish(off)


def main():
    rclpy.init()
    rclpy.spin(VisionNode())


if __name__ == "__main__":
    main()
