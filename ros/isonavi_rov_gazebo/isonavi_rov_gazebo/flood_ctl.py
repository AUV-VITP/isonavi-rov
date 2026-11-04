"""CLI command tool to activate, stop, or check flood conditions in isonavi-rov.

Usage:
  ros2 run isonavi_rov_gazebo flood_ctl start     # Starts river flood
  ros2 run isonavi_rov_gazebo flood_ctl stop      # Stops river flood
  ros2 run isonavi_rov_gazebo flood_ctl status    # Views flood status
"""

import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


def print_help():
    print("""
======================================================
         ISONAVI-ROV RIVER FLOOD CONTROL CLI
======================================================
  Usage:
    ros2 run isonavi_rov_gazebo flood_ctl start   -> Trigger continuous flood
    ros2 run isonavi_rov_gazebo flood_ctl stop    -> Stop flood & return to calm
    ros2 run isonavi_rov_gazebo flood_ctl status  -> View live river condition
======================================================
""")


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
        print("\n[SUCCESS] >>> FLOOD INITIATED! River current rising and flood water surging! <<<")
        print("          Run 'ros2 run isonavi_rov_gazebo flood_ctl stop' to halt the flood.\n")

    elif cmd in ("stop", "off", "disable", "0", "false"):
        pub = node.create_publisher(Bool, "flood", 10)
        t_end = time.time() + 1.5
        while node.count_subscribers("flood") == 0 and time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(5):
            pub.publish(Bool(data=False))
            rclpy.spin_once(node, timeout_sec=0.05)
        print("\n[SUCCESS] >>> FLOOD STOPPED! River conditions calming to normal. <<<\n")

    elif cmd in ("status", "info"):
        def on_status(msg):
            print(f"\n[RIVER TELEMETRY] {msg.data}\n")
            rclpy.shutdown()

        node.create_subscription(String, "flood_status", on_status, 10)
        print("Listening for river condition telemetry...")
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
