"""Gamepad to body wrench, with depth, heading and attitude hold.

Left stick surge and sway, right stick yaw and depth. In assist mode the yaw
and depth sticks move the held references; in manual mode they are thrust.
In auto mode, with centring engaged by the first button on entering a
corridor, the vehicle also centres itself from the camera's wall estimate,
with the lateral law the simulation flies; the pilot's sway stick still adds
to it. The same button releases it.

In shared mode the same button engages a blend instead: the sway command is
the pilot's stick and the camera's centring law, weighted by the inverse of
the error each has, as the simulation's shared mode does. The camera's error
is the wall estimate's calibrated error at the set water clarity, the worst
section's, since the node does not know which section it is in; the pilot's
is the operator model's read error at the set range and clarity. The default
range, 1.5 m, is the middle of the 3 m the operator model looks ahead along a
corridor leg.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Float64

from isonavi.control import wrap
from isonavi.dynamics import isonavi_rov
from isonavi.pilot import AssistController, PilotConfig, blend_weight, read_error
from isonavi.vision import load_errors


def quat_to_euler(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


class PilotNode(Node):
    def __init__(self):
        super().__init__("pilot")
        self.mode = self.declare_parameter("mode", "assist").value
        self.max_f = self.declare_parameter("max_force_n", 40.0).value
        self.rate = self.declare_parameter("rate_hz", 50.0).value
        p = isonavi_rov
        self.assist = AssistController(p.buoyancy - p.weight)
        self.att = np.zeros(3)
        self.gyro = np.zeros(3)
        self.z = None
        self.z_ref = None
        self.yaw_ref = None
        self.axes = [0.0] * 8
        self.offset = float("nan")
        self.offset_rate = 0.0
        self.offset_t = None
        self.centring = False
        self.button_prev = 0
        self.lat = PilotConfig()
        b = self.declare_parameter("scattering", 0.4).value
        rng = self.declare_parameter("read_range_m", 1.5).value
        err = load_errors(b)
        cam_err = max(err.walls.values()) if err is not None else 0.1
        self.cam_weight = float(blend_weight(read_error(self.lat, rng, b), cam_err))
        if self.mode == "shared":
            self.get_logger().info(f"shared mode: camera weight {self.cam_weight:.2f}")
        self.create_subscription(Joy, "joy", self.on_joy, 10)
        self.create_subscription(Float64, "corridor_offset", self.on_offset, 10)
        self.create_subscription(Imu, "imu", self.on_imu, 10)
        self.create_subscription(Float64, "depth", self.on_depth, 10)
        self.pub = self.create_publisher(Wrench, "wrench_cmd", 10)
        self.create_timer(1.0 / self.rate, self.tick)

    def on_joy(self, m):
        self.axes = list(m.axes) + [0.0] * (8 - len(m.axes))
        b = m.buttons[0] if len(m.buttons) else 0
        if b and not self.button_prev:
            self.centring = not self.centring
        self.button_prev = b

    def on_imu(self, m):
        self.att = quat_to_euler(m.orientation)
        self.gyro = np.array([m.angular_velocity.x, m.angular_velocity.y,
                              m.angular_velocity.z])

    def on_offset(self, m):
        # Differentiate at the camera rate, where the signal changes, not at
        # the control rate, where it would be a train of steps.
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.offset_t is not None and np.isfinite(self.offset) and t > self.offset_t:
            self.offset_rate = (m.data - self.offset) / (t - self.offset_t)
        self.offset, self.offset_t = m.data, t

    def on_depth(self, m):
        self.z = -abs(m.data)

    def tick(self):
        if self.z is None:
            return
        dt = 1.0 / self.rate
        ax = self.axes
        if self.z_ref is None:
            self.z_ref, self.yaw_ref = self.z, self.att[2]
        tau = np.zeros(6)
        tau[0], tau[1] = self.max_f * ax[1], self.max_f * ax[0]
        if self.mode == "manual":
            tau[2] = self.max_f * ax[4]
            tau[5] = 3.0 * ax[3]
        else:
            self.yaw_ref = wrap(self.yaw_ref + 0.8 * ax[3] * dt)
            self.z_ref = float(np.clip(self.z_ref + 0.3 * ax[4] * dt, -1.9, -0.1))
            hdg_err = wrap(self.yaw_ref - self.att[2])
            tau[2:] = self.assist.wrench(self.z_ref, self.z, self.att, self.gyro,
                                         hdg_err, dt)[2:]
            if self.mode in ("auto", "shared") and self.centring and np.isfinite(self.offset):
                lat = self.offset + self.lat.k_lat_d * self.offset_rate
                cam = float(np.clip(-self.lat.k_lat * lat, -40, 40))
                if self.mode == "auto":
                    tau[1] += cam
                else:
                    w = self.cam_weight
                    tau[1] = (1.0 - w) * tau[1] + w * cam
        w = Wrench()
        w.force.x, w.force.y, w.force.z = map(float, tau[:3])
        w.torque.x, w.torque.y, w.torque.z = map(float, tau[3:])
        self.pub.publish(w)


def main():
    rclpy.init()
    rclpy.spin(PilotNode())


if __name__ == "__main__":
    main()
