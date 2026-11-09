"""Start, stop or check the simulated flood current.

Usage:
  ros2 run isonavi_rov_gazebo flood_ctl start     # Starts the current
  ros2 run isonavi_rov_gazebo flood_ctl stop      # Stops the current
  ros2 run isonavi_rov_gazebo flood_ctl status    # Current flow
"""

import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


def print_help():
    print(__doc__)


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help", "help"):
        print_help()
        return

    cmd = sys.argv[1].lower()
    rclpy.init()
    node = Node("flood_ctl_cli")

    if cmd in ("start", "on", "enable", "1", "true"):
        pub = node.create_publisher(Bool, "flood", 10)
        t_end = time.time() + 1.5
        while node.count_subscribers("flood") == 0 and time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(5):
            pub.publish(Bool(data=True))
            rclpy.spin_once(node, timeout_sec=0.05)
        print("flood started")

    elif cmd in ("stop", "off", "disable", "0", "false"):
        pub = node.create_publisher(Bool, "flood", 10)
        t_end = time.time() + 1.5
        while node.count_subscribers("flood") == 0 and time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(5):
            pub.publish(Bool(data=False))
            rclpy.spin_once(node, timeout_sec=0.05)
        print("flood stopped")

    elif cmd in ("status", "info"):
        def on_status(msg):
            print(msg.data)
            rclpy.shutdown()

        node.create_subscription(String, "flood_status", on_status, 10)
        rclpy.spin(node)
        return
    else:
        print(f"Unknown command: {cmd}")
        print_help()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
