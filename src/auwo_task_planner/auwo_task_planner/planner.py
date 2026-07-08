#!/usr/bin/env python3
"""Planner + confirm gate (single robot, human-in-the-loop).

Reads the situational context, checks the excavation goal is satisfiable and the
e-stop is clear, proposes the dig->dump cycle to the human, and only dispatches
/excavate after an explicit confirm. CLI-driven: propose, then confirm.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from auwo_interfaces.srv import GetSituationalContext


class Planner(Node):
    def __init__(self):
        super().__init__("planner")
        self.cb = ReentrantCallbackGroup()

        self.declare_parameter("require_dump_reachable", True)

        self._estopped = False
        self._pending = False   # a proposed cycle is awaiting confirm

        self.create_subscription(Bool, "/estop/stopped", self._estop_cb, 10, callback_group=self.cb)
        self._ctx_cli = self.create_client(GetSituationalContext, "/get_situational_context",
                                           callback_group=self.cb)
        self._excavate_cli = self.create_client(Trigger, "/excavate", callback_group=self.cb)

        self.create_service(Trigger, "/excavation/propose", self._propose_cb, callback_group=self.cb)
        self.create_service(Trigger, "/excavation/confirm", self._confirm_cb, callback_group=self.cb)

        self.get_logger().info("Planner ready. Propose: ros2 service call /excavation/propose "
                               "std_srvs/srv/Trigger  then confirm: /excavation/confirm")

    def _estop_cb(self, msg: Bool):
        self._estopped = bool(msg.data)
        if self._estopped and self._pending:
            self._pending = False
            self.get_logger().warn("E-stop tripped — pending proposal cleared.")

    def _wait(self, future, timeout=5.0):
        start = time.time()
        while not future.done() and time.time()-start < timeout:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _get_context(self):
        if not self._ctx_cli.wait_for_service(timeout_sec=3.0):
            return None
        return self._wait(self._ctx_cli.call_async(GetSituationalContext.Request()))

    def _propose_cb(self, request, response):
        if self._estopped:
            response.success = False; response.message = "e-stopped — re-arm first"; return response

        resp = self._get_context()
        if resp is None:
            response.success = False; response.message = "situational context unavailable"; return response
        c = resp.context

        # --- goal feasibility check (the cheap precondition gate) ---
        if not c.has_dig:
            response.success = False; response.message = "no dig target in situational context"; return response
        if not c.dig_reachable:
            response.success = False; response.message = "dig target not reachable"; return response
        if self._p("require_dump_reachable") and c.has_dump and not c.dump_reachable:
            response.success = False; response.message = "dump target not reachable"; return response

        d = c.dig_target; p = c.dump_target
        self.get_logger().info("──────── PROPOSED EXCAVATION ────────")
        self.get_logger().info(f"  DIG  → ({d.x:.3f}, {d.y:.3f}, {d.z:.3f})  reachable={c.dig_reachable}")
        if c.has_dump:
            self.get_logger().info(f"  DUMP → ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})  reachable={c.dump_reachable}")
        self.get_logger().info("  Confirm to run:  ros2 service call /excavation/confirm std_srvs/srv/Trigger")
        self.get_logger().info("─────────────────────────────────────")

        self._pending = True
        response.success = True
        response.message = "proposed — awaiting confirm"
        return response

    def _confirm_cb(self, request, response):
        if self._estopped:
            self._pending = False
            response.success = False; response.message = "e-stopped — re-arm first"; return response
        if not self._pending:
            response.success = False; response.message = "nothing proposed — call /excavation/propose first"; return response

        self._pending = False
        self.get_logger().info("Confirmed — dispatching excavation cycle.")
        if not self._excavate_cli.wait_for_service(timeout_sec=3.0):
            response.success = False; response.message = "excavation server unavailable"; return response
        res = self._wait(self._excavate_cli.call_async(Trigger.Request()), timeout=120.0)
        if res is None:
            response.success = False; response.message = "excavate timed out / no result"; return response
        response.success = res.success
        response.message = f"excavate: {res.message}"
        return response

    def _p(self, n): return self.get_parameter(n).value


def main():
    rclpy.init()
    node = Planner()
    ex = MultiThreadedExecutor(); ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
