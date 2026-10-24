"""Per-thruster force to ESC pulse widths on the ESP32, over the framed protocol.

The ESP32 firmware is the one validated in the loop; it drops every channel
to neutral if this node goes quiet. Without a port the node only publishes
the widths it would send, which is how it is exercised on the bench.
"""

import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32MultiArray

from isonavi.dynamics import isonavi_rov

sys.path.insert(0, os.path.join(os.environ.get("ISONAVI_ROOT", ""), "hil", "common"))
try:
    import hil_protocol as P
except ImportError:
    P = None


def thrust_to_pwm(force_n, max_n):
    frac = float(np.clip(force_n / max_n, -1.0, 1.0))
    return int(round(1500 + frac * 400))


class ActuatorNode(Node):
    def __init__(self):
        super().__init__("actuator")
        port = self.declare_parameter("port", "").value
        self.ser = None
        if port and P is not None:
            import serial
            self.ser = serial.Serial(port, 115200, timeout=0)
        self.create_subscription(Float64MultiArray, "thrusters", self.on_thrust, 10)
        self.pub = self.create_publisher(Int32MultiArray, "pwm_us", 10)

    def on_thrust(self, m):
        w = [thrust_to_pwm(f, isonavi_rov.max_thrust_n) for f in m.data]
        if self.ser is not None:
            self.ser.write(P.encode(P.MSG_PWM, P.pack_pwm(w)))
        out = Int32MultiArray()
        out.data = w
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(ActuatorNode())


if __name__ == "__main__":
    main()
