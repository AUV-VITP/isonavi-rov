"""Gamepad to body wrench, with Dynamic Positioning (Station Keeping), depth, heading, and attitude hold.

In assist mode:
  - When pilot sticks are neutral: locks 3D world position (x, y, z) and heading (yaw),
    automatically commanding counter-thrust to reject river currents and flood disturbances.
  - When pilot pushes sticks: smoothly commands cruise velocity and moves the held references.
  - Auto/Shared modes maintain camera-assisted corridor / riverbank tracking.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Bool, Float64

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
        # Tuned gains: smooth depth and attitude hold without chattering or vibration
        self.assist = AssistController(
            p.buoyancy - p.weight,
            kz=75.0,        # Smooth depth stiffness (was 260.0 which vibrated)
            kvz=35.0,       # Damped vertical velocity
            kyaw=4.0,       # Stable yaw rate hold
            katt=8.0,       # Soft attitude damping respecting passive righting moment (was 40.0)
            katt_d=2.5,     # Low angular damping avoiding gyro sensor noise amplification (was 12.0)
            vz_tau=0.25,    # Smoother velocity filter
        )
        self.tau_smooth = np.zeros(6)
        self.att = np.zeros(3)
        self.gyro = np.zeros(3)

        # Odometry tracking & Station-Keeping references
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.odom_received = False
        self.x_ref = None
        self.y_ref = None

        self.z = None
        self.z_ref = None
        self.yaw_ref = None
        self.axes = [0.0] * 8
        self.offset = float("nan")
        self.offset_t = None
        self.offset_prev = None
        self.centring = False
        self.button_prev = 0
        self.flood_active = False

        self.lat = PilotConfig()
        b = self.declare_parameter("scattering", 0.4).value
        rng = self.declare_parameter("read_range_m", 1.5).value
        err = load_errors(b)
        cam_err = max(err.walls.values()) if err is not None else 0.1
        self.cam_weight = float(blend_weight(read_error(self.lat, rng, b), cam_err))
        if self.mode == "shared":
            self.get_logger().info(f"shared mode: camera weight {self.cam_weight:.2f}")

        # Subscriptions
        self.create_subscription(Joy, "joy", self.on_joy, 10)
        self.create_subscription(Float64, "corridor_offset", self.on_offset, 10)
        self.create_subscription(Imu, "imu", self.on_imu, 10)
        self.create_subscription(Float64, "depth", self.on_depth, 10)
        self.create_subscription(Odometry, "model/isonavi_rov/odometry", self.on_odom, 10)
        self.create_subscription(Bool, "flood_active", self.on_flood_active, 10)

        # Publishers
        self.pub = self.create_publisher(Wrench, "wrench_cmd", 10)
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info("PilotNode initialized with Active Dynamic Positioning (Station-Keeping).")

    def on_flood_active(self, m: Bool):
        self.flood_active = bool(m.data)

    def on_odom(self, m: Odometry):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.vx = m.twist.twist.linear.x
        self.vy = m.twist.twist.linear.y
        self.odom_received = True

    def on_joy(self, m):
        self.axes = list(m.axes) + [0.0] * (8 - len(m.axes))
        b = m.buttons[0] if len(m.buttons) else 0
        if b and not self.button_prev:
            self.centring = not self.centring
        self.button_prev = b

    def on_imu(self, m):
        self.att = quat_to_euler(m.orientation)
        self.gyro = np.array([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_offset(self, m):
        self.offset, self.offset_t = m.data, self.now()

    def on_depth(self, m):
        self.z = -abs(m.data)

    def tick(self):
        if self.z is None:
            return
        dt = 1.0 / self.rate
        ax = self.axes
        yaw = self.att[2]

        # Initialize held references upon first measurements
        if self.z_ref is None:
            self.z_ref = self.z
            self.yaw_ref = yaw

        if self.odom_received and self.x_ref is None:
            self.x_ref = self.x
            self.y_ref = self.y

        tau = np.zeros(6)

        # 1. Surge & Sway (Pilot Override vs Station-Keeping)
        surge_active = abs(ax[1]) > 0.05
        sway_active = abs(ax[0]) > 0.05

        if self.mode == "manual":
            tau[0] = self.max_f * ax[1]
            tau[1] = self.max_f * ax[0]
            tau[2] = self.max_f * ax[4]
            tau[5] = 3.0 * ax[3]
        else:
            # ASSIST / AUTO / SHARED MODE
            # A. Dynamic Positioning (Station-Keeping in X & Y)
            if self.odom_received and self.x_ref is not None:
                if surge_active or sway_active:
                    # Actively piloting: follow stick commands and update station setpoint
                    self.x_ref = self.x
                    self.y_ref = self.y
                    tau[0] = self.max_f * ax[1]
                    tau[1] = self.max_f * ax[0]
                else:
                    # Neutral stick: Hold position against river flow / flood current smoothly
                    ex_w = self.x_ref - self.x
                    ey_w = self.y_ref - self.y

                    # Transform error and velocity to ROV body frame
                    e_surge = ex_w * np.cos(yaw) + ey_w * np.sin(yaw)
                    e_sway = -ex_w * np.sin(yaw) + ey_w * np.cos(yaw)
                    v_surge = self.vx * np.cos(yaw) + self.vy * np.sin(yaw)
                    v_sway = -self.vx * np.sin(yaw) + self.vy * np.cos(yaw)

                    # 2 cm deadband to eliminate sensor noise micro-twitching
                    if abs(e_surge) < 0.02:
                        e_surge = 0.0
                    if abs(e_sway) < 0.02:
                        e_sway = 0.0

                    # Critically damped positioning gains
                    kp = 24.0 if self.flood_active else 16.0
                    kd = 14.0 if self.flood_active else 10.0

                    tau[0] = float(np.clip(kp * e_surge - kd * v_surge, -self.max_f, self.max_f))
                    tau[1] = float(np.clip(kp * e_sway - kd * v_sway, -self.max_f, self.max_f))
            else:
                tau[0] = self.max_f * ax[1]
                tau[1] = self.max_f * ax[0]

            # B. Heading and Depth Hold
            self.yaw_ref = wrap(self.yaw_ref + 0.8 * ax[3] * dt)
            self.z_ref = float(np.clip(self.z_ref + 0.3 * ax[4] * dt, -8.0, -0.1))
            hdg_err = wrap(self.yaw_ref - yaw)
            tau[2:] = self.assist.wrench(self.z_ref, self.z, self.att, self.gyro, hdg_err, dt)[2:]

            # C. Corridor / Wall offset centring (if active)
            fresh = (self.offset_t is not None and np.isfinite(self.offset)
                     and self.now() - self.offset_t <= 0.45)
            if not (self.mode in ("auto", "shared") and self.centring and fresh):
                self.offset_prev = None
            else:
                rate = 0.0 if self.offset_prev is None else (self.offset - self.offset_prev) / dt
                self.offset_prev = self.offset
                lat = self.offset + self.lat.k_lat_d * rate
                cam = float(np.clip(-self.lat.k_lat * lat, -40, 40))
                if self.mode == "auto":
                    tau[1] += cam
                else:
                    w = self.cam_weight
                    tau[1] = (1.0 - w) * tau[1] + w * cam

        # Low-pass filter to eliminate discrete 50 Hz thruster step chatter
        self.tau_smooth = 0.65 * self.tau_smooth + 0.35 * tau
        w = Wrench()
        w.force.x, w.force.y, w.force.z = map(float, self.tau_smooth[:3])
        w.torque.x, w.torque.y, w.torque.z = map(float, self.tau_smooth[3:])
        self.pub.publish(w)


def main():
    rclpy.init()
    node = PilotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
