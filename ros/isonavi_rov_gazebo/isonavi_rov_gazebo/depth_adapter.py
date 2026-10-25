"""Depth for the pilot node, as the vehicle's pressure cell would give it.

Gazebo Harmonic has no water pressure sensor: its air pressure sensor models
the atmosphere and reads about 12 Pa per metre of depth, where water gives
about 9.8 kPa. So depth is taken from the vehicle's pose, at the CG as the
python depth cell takes it, with that cell's noise and rate.

Subscribes model/isonavi_rov/odometry; publishes depth (std_msgs/Float64,
metres below the surface, positive).
"""

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

from isonavi.sensors import DepthConfig


class DepthAdapterNode(Node):
    def __init__(self):
        super().__init__("depth_adapter")
        self.cfg = DepthConfig()
        self.rng = np.random.default_rng(self.declare_parameter("seed", 2).value)
        self.next_t = 0.0
        self.pub = self.create_publisher(Float64, "depth", 10)
        self.create_subscription(Odometry, "model/isonavi_rov/odometry", self.on_odom, 10)

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t < self.next_t:
            return
        self.next_t = t + 1.0 / self.cfg.rate_hz
        out = Float64()
        out.data = float(-msg.pose.pose.position.z + self.rng.normal(0.0, self.cfg.sigma))
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(DepthAdapterNode())


if __name__ == "__main__":
    main()
