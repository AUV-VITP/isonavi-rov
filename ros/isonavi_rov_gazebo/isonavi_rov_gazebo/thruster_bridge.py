"""Splits multi-array thruster forces into individual Gazebo Harmonic joint topics.

Subscribes to `/thrusters` (std_msgs/Float64MultiArray from safety_node) and publishes
each channel to `/model/isonavi_rov/joint/thruster_X_joint/cmd_thrust` (std_msgs/Float64),
which are bridged to Gazebo Harmonic to drive the 8 thruster plugins.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray


class ThrusterBridgeNode(Node):
    def __init__(self):
        super().__init__("thruster_bridge")
        self.pubs = [
            self.create_publisher(
                Float64,
                f"/model/isonavi_rov/joint/thruster_{i + 1}_joint/cmd_thrust",
                10,
            )
            for i in range(8)
        ]
        self.sub = self.create_subscription(
            Float64MultiArray, "thrusters", self.on_thrusters, 10
        )
        self.get_logger().info("Thruster bridge active for 8 Gazebo Harmonic thrusters.")

    def on_thrusters(self, msg):
        forces = msg.data
        n = min(len(forces), 8)
        for i in range(n):
            f_msg = Float64()
            f_msg.data = float(forces[i])
            self.pubs[i].publish(f_msg)


def main():
    rclpy.init()
    rclpy.spin(ThrusterBridgeNode())


if __name__ == "__main__":
    main()
