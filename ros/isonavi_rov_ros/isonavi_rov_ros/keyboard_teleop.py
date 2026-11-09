"""Interactive keyboard teleoperation for isonavi-rov.

Publishes sensor_msgs/Joy at 20 Hz to drive pilot_node and satisfy the
safety supervisor heartbeat without requiring a physical gamepad.
"""

import select
import sys
import termios
import tty
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy

HELP_MSG = """
======================================================
         ISONAVI-ROV KEYBOARD TELEOPERATION
======================================================
  Surge (Forward / Back)  :  W / S   (+/- 0.25)
  Sway  (Port / Stbd)     :  A / D   (+/- 0.25)
  Yaw   (Turn Left / Right:  Q / E   (+/- 0.25)
  Heave (Ascend / Dive)   :  I / K   (+/- 0.20)
  Cruise Presets          :  1 (25%), 2 (50%), 3 (75%), 4 (100%)
  All Stop (Neutral)      :  Space
  Toggle Auto-Centring    :  C
  Quit Teleoperation      :  Ctrl+C or X
======================================================
"""


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.pub = self.create_publisher(Joy, "joy", 10)
        self.surge = 0.0
        self.sway = 0.0
        self.yaw = 0.0
        self.heave = 0.0
        self.centring = False
        self.press_ticks = 0    # the pilot toggles centring on a rising edge
        self.timer = self.create_timer(0.05, self.publish_joy)  # 20 Hz heartbeat
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        print(HELP_MSG)
        self.print_status()

    def print_status(self):
        c_state = "ON " if self.centring else "OFF"
        status = (
            f"\r[Surge: {self.surge:+.2f} | Sway: {self.sway:+.2f} | "
            f"Yaw: {self.yaw:+.2f} | Heave: {self.heave:+.2f} | Auto: {c_state}] "
        )
        sys.stdout.write(status)
        sys.stdout.flush()

    def publish_joy(self):
        # Poll keyboard input non-blocking
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = sys.stdin.read(1).lower()
            if key == "w":
                self.surge = min(1.0, self.surge + 0.25)
            elif key == "s":
                self.surge = max(-1.0, self.surge - 0.25)
            elif key == "a":
                self.sway = min(1.0, self.sway + 0.25)
            elif key == "d":
                self.sway = max(-1.0, self.sway - 0.25)
            elif key == "q":
                self.yaw = min(1.0, self.yaw + 0.25)
            elif key == "e":
                self.yaw = max(-1.0, self.yaw - 0.25)
            elif key == "i":
                self.heave = min(1.0, self.heave + 0.20)
            elif key == "k":
                self.heave = max(-1.0, self.heave - 0.20)
            elif key == "1":
                self.surge = 0.25
            elif key == "2":
                self.surge = 0.50
            elif key == "3":
                self.surge = 0.75
            elif key == "4":
                self.surge = 1.00
            elif key == " ":
                self.surge = 0.0
                self.sway = 0.0
                self.yaw = 0.0
                self.heave = 0.0
            elif key == "c":
                self.centring = not self.centring
                self.press_ticks = 2
            elif key in ("x", "\x03"):
                print("\nExiting teleoperation...")
                self.surge = self.sway = self.yaw = self.heave = 0.0
                self.send_msg()
                rclpy.shutdown()
                return

            self.print_status()

        self.send_msg()

    def send_msg(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        # axes: [0: sway, 1: surge, 2: unused, 3: yaw, 4: depth, 5..7: unused]
        msg.axes = [
            float(self.sway),
            float(self.surge),
            0.0,
            float(self.yaw),
            float(self.heave),
            0.0,
            0.0,
            0.0,
        ]
        msg.buttons = [1 if self.press_ticks > 0 else 0, 0, 0, 0, 0, 0, 0, 0]
        self.press_ticks = max(0, self.press_ticks - 1)
        self.pub.publish(msg)

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main():
    rclpy.init()
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
