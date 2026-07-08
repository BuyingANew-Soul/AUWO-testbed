#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time

from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
import tf2_ros

ARM_GROUP="arm"; GRIPPER_GROUP="gripper"; GRIPPER_JOINT="link3_to_gripper_link"
EE_LINK="hand_tcp"; PLANNING_FRAME="base_link"


class ExcavationServer(Node):
    def __init__(self):
        super().__init__("excavation_server")
        self.cb = ReentrantCallbackGroup()

        self.declare_parameter("dig_x", 0.22)
        self.declare_parameter("dig_y", 0.0)
        self.declare_parameter("dig_z", 0.04)
        self.declare_parameter("z_floor", -0.10)
        self.declare_parameter("approach_height", 0.08)
        self.declare_parameter("drag_distance", 0.03)
        self.declare_parameter("lift_height", 0.10)
        self.declare_parameter("dump_x", 0.20)
        self.declare_parameter("dump_y", 0.12)
        self.declare_parameter("dump_z", 0.06)
        self.declare_parameter("dump_hover", 0.06)
        self.declare_parameter("home_x", 0.18)
        self.declare_parameter("home_y", 0.0)
        self.declare_parameter("home_z", 0.12)
        self.declare_parameter("gripper_open_rad", 0.0)
        self.declare_parameter("gripper_grip_rad", 1.5)
        self.declare_parameter("vel_scale", 0.3)
        self.declare_parameter("ik_avoid_collisions", True)

        self._target = None
        self._dump_target = None
        self._busy = False

        # --- e-stop state ---
        self._estopped = False
        self._active_goal = None   # in-flight MoveGroup goal handle, for cancellation

        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        self.create_subscription(PointStamped, "/excavation/target", self._target_cb, 10, callback_group=self.cb)
        self.create_subscription(PointStamped, "/excavation/dump_target", self._dump_cb, 10, callback_group=self.cb)
        self.create_subscription(Bool, "/estop/stopped", self._estop_cb, 10, callback_group=self.cb)
        self._move = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb)
        self._ik_cli = self.create_client(GetPositionIK, "/compute_ik", callback_group=self.cb)
        self.create_service(Trigger, "excavate", self._excavate_cb, callback_group=self.cb)
        self.get_logger().info("Excavation server ready. Trigger with: ros2 service call /excavate std_srvs/srv/Trigger")

    def _p(self, n): return self.get_parameter(n).value

    def _floor(self, z): return max(float(z), self._p("z_floor"))

    def _estop_cb(self, msg: Bool):
        was = self._estopped
        self._estopped = bool(msg.data)
        if self._estopped and not was:
            self.get_logger().error("E-STOP received — cancelling any in-flight motion.")
            self._cancel_active()

    def _cancel_active(self):
        gh = self._active_goal
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"goal cancel failed: {e}")

    def _target_cb(self, msg):
        self._target = (msg.point.x, msg.point.y, msg.point.z)

    def _dump_cb(self, msg):
        self._dump_target = (msg.point.x, msg.point.y, msg.point.z)

    def _dig_point(self):
        if self._target is not None:
            x, y, z = self._target
        else:
            x, y, z = self._p("dig_x"), self._p("dig_y"), self._p("dig_z")
        return (x, y, self._floor(z))

    def _dump_point(self):
        if self._dump_target is not None:
            x, y, z = self._dump_target
        else:
            x, y, z = self._p("dump_x"), self._p("dump_y"), self._p("dump_z")
        return (x, y, self._floor(z))

    def _wait(self, future, timeout=40.0):
        start = time.time()
        while not future.done() and time.time()-start < timeout:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def _log_tcp(self, cx, cy, cz):
        time.sleep(0.15)
        try:
            tr = self._tf_buf.lookup_transform(PLANNING_FRAME, EE_LINK, Time())
            a = tr.transform.translation
            self.get_logger().info(
                f"    TCP cmd=({cx:.3f},{cy:.3f},{cz:.3f}) "
                f"actual=({a.x:.3f},{a.y:.3f},{a.z:.3f}) "
                f"err=({a.x-cx:+.3f},{a.y-cy:+.3f},{a.z-cz:+.3f})")
        except Exception as e:
            self.get_logger().warn(f"    TCP lookup ({PLANNING_FRAME}->{EE_LINK}) failed: {e}")

    def _ik(self, x, y, z):
        if not self._ik_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_ik not available"); return None
        req = GetPositionIK.Request(); ik = req.ik_request
        ik.group_name = ARM_GROUP; ik.ik_link_name = EE_LINK
        ik.avoid_collisions = bool(self._p("ik_avoid_collisions"))
        ik.pose_stamped.header.frame_id = PLANNING_FRAME
        ik.pose_stamped.pose.position.x = float(x)
        ik.pose_stamped.pose.position.y = float(y)
        ik.pose_stamped.pose.position.z = float(z)
        ik.pose_stamped.pose.orientation.w = 1.0
        ik.timeout.sec = 1
        resp = self._wait(self._ik_cli.call_async(req))
        if resp is None:
            self.get_logger().error("compute_ik timed out"); return None
        if resp.error_code.val != 1:
            self.get_logger().warn(f"IK FAILED at ({x:.3f},{y:.3f},{z:.3f}) code={resp.error_code.val} (unreachable)")
            return None
        js = resp.solution.joint_state
        return {n: p for n, p in zip(js.name, js.position) if n != GRIPPER_JOINT}

    def _move_arm(self, x, y, z):
        if self._estopped:
            return False
        sol = self._ik(x, y, z)
        if sol is None: return False
        c = Constraints()
        for n, p in sol.items():
            jc = JointConstraint(); jc.joint_name = n; jc.position = float(p)
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
            c.joint_constraints.append(jc)
        ok = self._call_move(c, ARM_GROUP)
        if ok:
            self._log_tcp(x, y, z)
        return ok

    def _move_gripper(self, v):
        if self._estopped:
            return False
        jc = JointConstraint(); jc.joint_name = GRIPPER_JOINT; jc.position = float(v)
        jc.tolerance_above = 0.02; jc.tolerance_below = 0.02; jc.weight = 1.0
        c = Constraints(); c.joint_constraints.append(jc)
        return self._call_move(c, GRIPPER_GROUP)

    def _call_move(self, constraints, group):
        if self._estopped:
            return False
        req = MotionPlanRequest(); req.group_name = group
        req.num_planning_attempts = 10; req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self._p("vel_scale")
        req.max_acceleration_scaling_factor = self._p("vel_scale")
        req.goal_constraints.append(constraints)
        g = MoveGroup.Goal(); g.request = req
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        g.planning_options.plan_only = False
        if not self._move.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("move_group not available"); return False
        gh = self._wait(self._move.send_goal_async(g))
        if gh is None or not gh.accepted:
            self.get_logger().error("move_group rejected goal"); return False
        self._active_goal = gh
        if self._estopped:                  # tripped during goal submission
            self._cancel_active()
        res = self._wait(gh.get_result_async())
        self._active_goal = None
        if res is None:
            self.get_logger().error("move_group timed out / cancelled"); return False
        if res.result.error_code.val != 1:
            self.get_logger().warn(f"move ended code={res.result.error_code.val} (cancelled or failed)")
            return False
        return True

    def _excavate_cb(self, request, response):
        if self._estopped:
            response.success = False; response.message = "e-stopped — re-arm to run"; return response
        if self._busy:
            response.success = False; response.message = "busy"; return response
        self._busy = True
        try:
            ok, msg = self._run_cycle()
            response.success = ok; response.message = msg
        finally:
            self._busy = False
        return response

    def _run_cycle(self):
        dx, dy, dz = self._dig_point()
        ah = self._p("approach_height"); drag = self._p("drag_distance"); lift = self._p("lift_height")
        px, py, pz = self._dump_point(); dh = self._p("dump_hover")
        hx, hy, hz = self._p("home_x"), self._p("home_y"), self._p("home_z")
        op = self._p("gripper_open_rad"); grip = self._p("gripper_grip_rad")

        self.get_logger().info(f"[excavate] dig ({dx:.3f},{dy:.3f},{dz:.3f}) → dump ({px:.3f},{py:.3f},{pz:.3f})")

        seq = [
            ("APPROACH",   lambda: self._move_arm(dx, dy, dz + ah)),
            ("BUCKET OPEN",lambda: self._move_gripper(op)),
            ("DIG",        lambda: self._move_arm(dx, dy, dz)),
            ("DRAG",       lambda: self._move_arm(dx - drag, dy, dz)),
            ("SCOOP",      lambda: self._move_gripper(grip)),
            ("LIFT",       lambda: self._move_arm(dx - drag, dy, dz + lift)),
            ("SWING",      lambda: self._move_arm(px, py, pz + dh)),
            ("DUMP DOWN",  lambda: self._move_arm(px, py, pz)),
            ("DUMP",       lambda: self._move_gripper(op)),
            ("RETURN",     lambda: self._move_arm(hx, hy, hz)),
        ]
        for name, fn in seq:
            if self._estopped:
                self.get_logger().error(f"[excavate] HALTED at {name} (e-stop)")
                return False, "e-stopped mid-cycle"
            self.get_logger().info(f"[excavate] {name}")
            if not fn():
                if self._estopped:
                    return False, "e-stopped mid-cycle"
                return False, f"{name} failed"
        self.get_logger().info("[excavate] cycle complete")
        return True, "excavation cycle complete"


def main():
    rclpy.init()
    node = ExcavationServer()
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
