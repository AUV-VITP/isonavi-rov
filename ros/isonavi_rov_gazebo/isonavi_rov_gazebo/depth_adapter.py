"""Converts Gazebo pressure or odometry to standard `/depth` (std_msgs/Float64).

Listens to:
  - `fluid_pressure` (sensor_msgs/FluidPressure)
  - `model/isonavi_rov/odometry` (nav_msgs/Odometry)

Publishes:
  - `depth` (std_msgs/Float64, positive magnitude or negative z)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

RHO = 1000.0
G = 9.80665
P_ATM = 101325.0


class DepthAdapterNode(Node):
    def __init__(self):
        super().__init__("depth_adapter")
        self.pub = self.create_publisher(Float64, "depth", 10)
        self.create_subscription(FluidPressure, "fluid_pressure", self.on_pressure, 10)
        self.create_subscription(Odometry, "model/isonavi_rov/odometry", self.on_odom, 10)
        self.last_z = 0.0
        self.get_logger().info("Depth adapter active.")

    def on_pressure(self, msg):
        # Hydrostatic depth h = (P - Patm) / (rho * g)
        p = msg.fluid_pressure
        h = max(0.0, (p - P_ATM) / (RHO * G))
        out = Float64()
        out.data = float(h)
        self.pub.publish(out)

    def on_odom(self, msg):
        # In ROS/Gazebo coordinates, z is negative when submerged below water surface at z=0
        z = msg.pose.pose.position.z
        h = abs(z)
        out = Float64()
        out.data = float(h)
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(DepthAdapterNode())


if __name__ == "__main__":
    main()
