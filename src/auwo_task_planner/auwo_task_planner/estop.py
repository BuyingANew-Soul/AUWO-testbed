#!/usr/bin/env python3
"""E-stop interlock: the single authority for the stop state.

Owns a latched 'stopped' boolean. State changes ONLY through the trip / rearm
services, so the latching and deliberate-re-arm rules are enforced by
construction (a stray topic publish can't clear it). The state is broadcast on
a latched topic for fast fan-out to anything that must obey (excavation server
now, arm driver later). This is a best-effort SOFTWARE stop, not a substitute
for a hardware e-stop that cuts motor power.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class EStop(Node):
    def __init__(self):
        super().__init__("estop")
        self.declare_parameter("start_stopped", False)  # arm tripped on startup if true
        self._stopped = bool(self.get_parameter("start_stopped").value)

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self._pub = self.create_publisher(Bool, "/estop/stopped", latched)

        self.create_service(Trigger, "/estop/trip", self._trip_cb)
        self.create_service(Trigger, "/estop/rearm", self._rearm_cb)

        self._broadcast()
        self.get_logger().warn("E-STOP TRIPPED — arm halted." if self._stopped
                               else "E-stop armed and clear. Trip: ros2 service call /estop/trip std_srvs/srv/Trigger")

    def _broadcast(self):
        self._pub.publish(Bool(data=self._stopped))

    def _trip_cb(self, request, response):
        if not self._stopped:
            self._stopped = True
            self._broadcast()
            self.get_logger().error("E-STOP TRIPPED — motion halted. Re-arm deliberately to resume.")
        response.success = True
        response.message = "stopped"
        return response

    def _rearm_cb(self, request, response):
        # deliberate re-arm: clears the latch and resumes readiness
        if self._stopped:
            self._stopped = False
            self._broadcast()
            self.get_logger().warn("E-stop RE-ARMED — system clear.")
        response.success = True
        response.message = "armed"
        return response


def main():
    rclpy.init()
    node = EStop()
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
