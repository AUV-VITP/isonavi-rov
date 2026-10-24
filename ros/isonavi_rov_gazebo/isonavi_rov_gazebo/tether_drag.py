"""Simulates the broadside hydrodynamic drag of trailing tether behind isonavi-rov.

Drag formula: F = 0.5 * rho * Cd * d * L_trail * v^2
At 0.35 m/s with 8 m trailing tether, adds approx 9.4 N of drag.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench

RHO = 1000.0
CD = 1.2
DIAMETER = 0.016  # 16 mm umbilical
SLACK_FRACTION = 0.35
ENTRY_POINT = (0.0, 0.0)  # tether surface entry point


class TetherDragNode(Node):
    def __init__(self):
        super().__init__("tether_drag")
        self.sub = self.create_subscription(Odometry, "model/isonavi_rov/odometry", self.on_odom, 10)
        self.pub = self.create_publisher(Wrench, "model/isonavi_rov/wrench", 10)
        self.get_logger().info("Tether drag simulation active.")

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        v_norm = math.hypot(vel.x, vel.y)
        if v_norm < 1e-3:
            return

        dist_from_entry = math.hypot(pos.x - ENTRY_POINT[0], pos.y - ENTRY_POINT[1])
        trail_m = max(2.0, min(15.0, dist_from_entry * SLACK_FRACTION))

        # Drag force opposing velocity direction
        f_drag = 0.5 * RHO * CD * DIAMETER * trail_m * (v_norm ** 2)

        w = Wrench()
        w.force.x = -f_drag * (vel.x / v_norm)
        w.force.y = -f_drag * (vel.y / v_norm)
        w.force.z = 0.0
        self.pub.publish(w)


def main():
    rclpy.init()
    rclpy.spin(TetherDragNode())


if __name__ == "__main__":
    main()
