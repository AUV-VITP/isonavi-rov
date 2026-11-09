"""Water current for the simulated flood.

The current goes to Gazebo's hydrodynamics on /ocean_current, which works out
the drag from the vehicle's speed through the water. Set it above the
vehicle's top speed and the vehicle is carried downstream, as it would be.

Subscribes flood (std_msgs/Bool). Publishes ocean_current
(geometry_msgs/Vector3, world frame), flood_active (std_msgs/Bool) and
flood_status (std_msgs/String).
"""

import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

TAU_S = 2.5     # time constant of the current building up and dying away
RATE_HZ = 10.0


class FloodControllerNode(Node):
    def __init__(self):
        super().__init__("flood_controller")
        self.max_flow = self.declare_parameter("max_flow_ms", 1.4).value
        self.requested = False
        self.flow = 0.0
        self.create_subscription(Bool, "flood", self.on_flood, 10)
        self.pub_current = self.create_publisher(Vector3, "ocean_current", 10)
        self.pub_active = self.create_publisher(Bool, "flood_active", 10)
        self.pub_status = self.create_publisher(String, "flood_status", 10)
        self.create_timer(1.0 / RATE_HZ, self.tick)

    def on_flood(self, msg):
        self.requested = msg.data

    def tick(self):
        target = self.max_flow if self.requested else 0.0
        self.flow += (target - self.flow) / (TAU_S * RATE_HZ)
        self.pub_current.publish(Vector3(x=float(self.flow)))
        active = self.flow > 0.1
        self.pub_active.publish(Bool(data=active))
        state = "FLOOD" if active else "CALM"
        self.pub_status.publish(String(data=f"{state} | current {self.flow:.2f} m/s"))


def main():
    rclpy.init()
    node = FloodControllerNode()
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
