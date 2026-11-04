"""Dynamic River Flood Simulation Controller for isonavi-rov.

Controls:
  - Subscribes to `/flood` (std_msgs/msg/Bool):
      True  -> Activates continuous river flood (rising water, turbulent current, mud surge).
      False -> Deactivates flood and returns river to baseline calm flow.

Publishes:
  - `/flood_status`       (std_msgs/msg/String) - Real-time telemetry (flow rate, water rise, status)
  - `/flood_active`       (std_msgs/msg/Bool)   - High-level flood state for vision and safety nodes
  - `/flood_water_rise`   (std_msgs/msg/Float64)- Surface water level rise (m) for depth adapter
  - `/flood_wrench`       (geometry_msgs/msg/Wrench) - Hydrodynamic river flood disturbance force
"""

import math
import rclpy
from geometry_msgs.msg import Wrench
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String


class FloodControllerNode(Node):
    def __init__(self):
        super().__init__("flood_controller")

        # Declare parameters
        self.flood_flow_ms = self.declare_parameter("max_flow_ms", 1.4).value
        self.flood_rise_m = self.declare_parameter("max_rise_m", 1.5).value
        self.flood_force_n = self.declare_parameter("max_drag_force_n", 32.0).value

        # Flood state variables
        self.flood_requested = False
        self.current_flow_ms = 0.0
        self.current_rise_m = 0.0
        self.current_force_n = 0.0
        self.sim_time = 0.0

        # Subscriptions
        self.sub_flood = self.create_subscription(Bool, "flood", self.on_flood_cmd, 10)

        # Publishers
        self.pub_status = self.create_publisher(String, "flood_status", 10)
        self.pub_active = self.create_publisher(Bool, "flood_active", 10)
        self.pub_rise = self.create_publisher(Float64, "flood_water_rise", 10)
        self.pub_wrench = self.create_publisher(Wrench, "flood_wrench", 10)

        # 50 Hz physics update loop & 1 Hz telemetry logger
        self.timer_physics = self.create_timer(0.02, self.update_physics)
        self.timer_log = self.create_timer(1.0, self.log_status)

        self.get_logger().info("FloodController initialized. Ready for flood commands on /flood topic.")

    def on_flood_cmd(self, msg: Bool):
        self.flood_requested = msg.data
        if self.flood_requested:
            self.get_logger().warn(">>> FLOOD ACTIVATED! River current rising, water level swelling, suspended silt surging! <<<")
        else:
            self.get_logger().info(">>> Flood stopping. River returning to baseline calm flow. <<<")

    def update_physics(self):
        dt = 0.02
        self.sim_time += dt

        # Smoothly ramp flood parameters in/out
        target_flow = self.flood_flow_ms if self.flood_requested else 0.0
        target_rise = self.flood_rise_m if self.flood_requested else 0.0
        target_force = self.flood_force_n if self.flood_requested else 0.0

        ramp_rate = 0.4 * dt  # Ramps over ~2.5 seconds
        self.current_flow_ms += (target_flow - self.current_flow_ms) * ramp_rate
        self.current_rise_m += (target_rise - self.current_rise_m) * ramp_rate
        self.current_force_n += (target_force - self.current_force_n) * ramp_rate

        # Calculate turbulent river flow with swirling eddies and surge waves
        t = self.sim_time
        surge_wave = 4.0 * math.sin(0.8 * t) + 2.0 * math.sin(1.7 * t)
        swirl_eddy = 2.5 * math.sin(0.5 * t) + 1.2 * math.sin(1.3 * t)

        # Total hydrodynamic downstream force
        fx = self.current_force_n + (surge_wave if self.current_force_n > 1.0 else 0.0)
        fy = swirl_eddy if self.current_force_n > 1.0 else 0.0

        # Publish flood wrench disturbance
        w = Wrench()
        w.force.x = float(fx)
        w.force.y = float(fy)
        w.force.z = 0.0
        self.pub_wrench.publish(w)

        # Publish water level rise & active state
        self.pub_rise.publish(Float64(data=float(self.current_rise_m)))
        self.pub_active.publish(Bool(data=(self.current_flow_ms > 0.1)))

    def log_status(self):
        is_active = self.current_flow_ms > 0.1
        status_str = (
            f"FLOOD {'ACTIVE [TORRENT]' if is_active else 'NORMAL [CALM]'} | "
            f"Flow: {self.current_flow_ms:.2f} m/s | "
            f"Water Level Rise: +{self.current_rise_m:.2f} m | "
            f"Disturbance Force: {self.current_force_n:.1f} N"
        )
        self.pub_status.publish(String(data=status_str))
        if is_active:
            self.get_logger().info(f"[FLOOD MONITOR] {status_str}")


def main():
    rclpy.init()
    node = FloodControllerNode()
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
