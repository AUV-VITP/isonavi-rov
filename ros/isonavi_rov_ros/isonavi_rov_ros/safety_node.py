"""Wrench to per-thruster force, through the power governor and the supervisor.

The pilot link heartbeat is the joystick topic: if it stops, the supervisor
holds zero thrust and the vehicle rises on its own buoyancy. The kill and
leak inputs latch until the node is restarted. 
"""

import os
import sys

ROOT_SIM = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../simulation"))
if ROOT_SIM not in sys.path:
    sys.path.insert(0, ROOT_SIM)

import numpy as np
try:
    import rclpy
    from geometry_msgs.msg import Wrench
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool, Float64MultiArray, String
except ImportError:
    pass

from isonavi.dynamics import VehicleDynamics, isonavi_rov
from isonavi.safety import PowerGovernor, Supervisor


class SafetyNode(Node):
    def __init__(self):
        super().__init__("safety")
        self.dyn = VehicleDynamics(isonavi_rov)
        self.gov = PowerGovernor()
        self.sup = Supervisor(self.declare_parameter("comms_timeout_s", 0.5).value)
        self.kill = self.leak = False
        self.f = np.zeros(8)
        self.create_subscription(Wrench, "wrench_cmd", self.on_wrench, 10)
        self.create_subscription(Joy, "joy", self.on_joy, 10)
        self.create_subscription(Bool, "kill", self.on_kill, 10)
        self.create_subscription(Bool, "leak", self.on_leak, 10)
        self.pub = self.create_publisher(Float64MultiArray, "thrusters", 10)
        self.state_pub = self.create_publisher(String, "safety_state", 10)
        self.sup.pilot_packet(self.now())
        self.create_timer(0.02, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joy(self, _):
        self.sup.pilot_packet(self.now())

    def on_kill(self, m):
        self.kill = self.kill or bool(m.data)

    def on_leak(self, m):
        self.leak = self.leak or bool(m.data)

    def on_wrench(self, w):
        tau = [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]
        self.f, _ = self.gov.limit(self.dyn.allocate(tau))

    def tick(self):
        state = self.sup.update(self.now(), kill=self.kill, leak=self.leak)
        msg = Float64MultiArray()
        msg.data = [float(x) for x in self.sup.gate(self.f)]
        self.pub.publish(msg)
        s = String()
        s.data = state if not self.sup.reason else f"{state}: {self.sup.reason}"
        self.state_pub.publish(s)


def main():
    rclpy.init()
    rclpy.spin(SafetyNode())


if __name__ == "__main__":
    main()
