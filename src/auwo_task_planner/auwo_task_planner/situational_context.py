#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, Point
from auwo_interfaces.msg import SituationalContext
from auwo_interfaces.srv import GetSituationalContext


class SituationalContextModel(Node):
    """Current situational context for excavation (DUL Situational Context Model).

    Holds the latest dig/dump targets (base_link), annotates each with a cheap
    reach-band check, and serves the consolidated context both as a latched topic
    and an on-demand service. The Action's IK remains the authoritative reach test.

    Today the dig/dump points arrive straight from sim_target_bridge, standing in
    for a future Environment Model + perception + derive step.
    """

    def __init__(self):
        super().__init__("situational_context")

        # cheap reach band (base_link). Generous on purpose; IK is authoritative.
        self.declare_parameter("reach_r_min", 0.10)
        self.declare_parameter("reach_r_max", 0.50)
        self.declare_parameter("reach_z_min", -0.15)
        self.declare_parameter("reach_z_max", 0.40)
        self.declare_parameter("dig_in_topic", "/excavation/target")
        self.declare_parameter("dump_in_topic", "/excavation/dump_target")
        self.declare_parameter("out_topic", "/auwo/situational_context")

        self._dig = None    # geometry_msgs/Point or None
        self._dump = None

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self._pub = self.create_publisher(SituationalContext, self._p("out_topic"), latched)
        self.create_subscription(PointStamped, self._p("dig_in_topic"), self._dig_cb, 10)
        self.create_subscription(PointStamped, self._p("dump_in_topic"), self._dump_cb, 10)
        self.create_service(GetSituationalContext, "get_situational_context", self._srv_cb)

        self._publish()  # initial (empty) context so late subscribers get something
        self.get_logger().info("Situational context up. Latched on "
                               f"{self._p('out_topic')}, service /get_situational_context")

    def _p(self, n): return self.get_parameter(n).value

    def _reachable(self, pt: Point) -> bool:
        r = math.hypot(pt.x, pt.y)
        return (self._p("reach_r_min") <= r <= self._p("reach_r_max")
                and self._p("reach_z_min") <= pt.z <= self._p("reach_z_max"))

    def _dig_cb(self, msg: PointStamped):
        self._dig = msg.point
        self._publish()

    def _dump_cb(self, msg: PointStamped):
        self._dump = msg.point
        self._publish()

    def _build(self) -> SituationalContext:
        sc = SituationalContext()
        sc.header.stamp = self.get_clock().now().to_msg()
        sc.header.frame_id = "base_link"
        sc.has_dig = self._dig is not None
        if self._dig is not None:
            sc.dig_target = self._dig
            sc.dig_reachable = self._reachable(self._dig)
        sc.has_dump = self._dump is not None
        if self._dump is not None:
            sc.dump_target = self._dump
            sc.dump_reachable = self._reachable(self._dump)
        return sc

    def _publish(self):
        self._pub.publish(self._build())

    def _srv_cb(self, request, response):
        response.context = self._build()
        return response


def main():
    rclpy.init()
    node = SituationalContextModel()
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
