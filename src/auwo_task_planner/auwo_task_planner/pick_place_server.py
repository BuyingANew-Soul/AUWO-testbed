#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from auwo_interfaces.action import PickObject
from auwo_interfaces.msg import WorldModel
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint

ARM_GROUP      = "arm"
GRIPPER_GROUP  = "gripper"
GRIPPER_JOINT  = "link3_to_gripper_link"
EE_LINK        = "hand_tcp"
PLANNING_FRAME = "base_link"


class PickPlaceServer(Node):
    def __init__(self):
        super().__init__("pick_place_server")
        self.cb = ReentrantCallbackGroup()

        self.declare_parameter("gripper_open_rad", 1.5)             # ~86 deg
        self.declare_parameter("gripper_grip_rad", math.radians(50))
        self.declare_parameter("hover_height", 0.00)
        self.declare_parameter("grasp_z_offset", 0.0)
        self.declare_parameter("lift_height", 0.08)
        self.declare_parameter("place_hover", 0.03)
        self.declare_parameter("vel_scale", 0.35)
        self.declare_parameter("ik_avoid_collisions", True)
        self.declare_parameter("grasp_dx", 0.0)   # +x = deeper into the box, away from camera
        self.declare_parameter("grasp_dy", 0.0)   # lateral
        self.declare_parameter("grasp_dz", 0.0)   # + = higher

        self._objects = {}
        self._busy = False
        self.create_subscription(WorldModel, "/auwo/world_model",
                                 self._wm_cb, 10, callback_group=self.cb)
        self._move = ActionClient(self, MoveGroup, "/move_action",
                                  callback_group=self.cb)
        self._ik_cli = self.create_client(GetPositionIK, "/compute_ik",
                                          callback_group=self.cb)
        self._server = ActionServer(self, PickObject, "pick_object",
                                    execute_callback=self._execute,
                                    goal_callback=self._goal_cb,
                                    callback_group=self.cb)
        self.get_logger().info("PickObject server ready on /pick_object.")

    def _p(self, name):
        return self.get_parameter(name).value

    def _wm_cb(self, msg):
        objs = None
        for f in ("objects", "detections", "detected_objects", "items"):
            if hasattr(msg, f):
                objs = getattr(msg, f); break
        if objs is not None:
            self._objects = {o.label: o for o in objs}

    def _goal_cb(self, goal_request):
        if self._busy:
            self.get_logger().warn("Busy with another pick; rejecting goal.")
            return GoalResponse.REJECT
        self._busy = True
        return GoalResponse.ACCEPT

    def _wait(self, future, timeout=40.0):
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.02)
        return future.result() if future.done() else None

    # ---- resolve a Cartesian target to a joint config via IKFast ----
    def _ik(self, x, y, z):
        if not self._ik_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/compute_ik not available")
            return None
        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = ARM_GROUP
        ik.ik_link_name = EE_LINK
        ik.avoid_collisions = bool(self._p("ik_avoid_collisions"))
        ik.pose_stamped.header.frame_id = PLANNING_FRAME
        ik.pose_stamped.pose.position.x = float(x)
        ik.pose_stamped.pose.position.y = float(y)
        ik.pose_stamped.pose.position.z = float(z)
        ik.pose_stamped.pose.orientation.w = 1.0   # ignored by translation3d solver
        ik.timeout.sec = 1
        resp = self._wait(self._ik_cli.call_async(req))
        if resp is None:
            self.get_logger().error("compute_ik timed out")
            return None
        if resp.error_code.val != 1:
            self.get_logger().warn(
                f"IK FAILED at ({x:.3f},{y:.3f},{z:.3f}) code={resp.error_code.val} "
                "(unreachable or in collision)")
            return None
        js = resp.solution.joint_state
        return {n: p for n, p in zip(js.name, js.position) if n != GRIPPER_JOINT}

    def _move_arm(self, x, y, z):
        sol = self._ik(x, y, z)
        if sol is None:
            return False
        return self._move_joints(sol)

    def _move_joints(self, joint_dict):
        c = Constraints()
        for name, pos in joint_dict.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return self._call_move(c)

    def _move_gripper(self, value_rad):
        jc = JointConstraint()
        jc.joint_name = GRIPPER_JOINT
        jc.position = float(value_rad)
        jc.tolerance_above = 0.02
        jc.tolerance_below = 0.02
        jc.weight = 1.0
        c = Constraints(); c.joint_constraints.append(jc)
        return self._call_move(c, group=GRIPPER_GROUP)

    def _call_move(self, constraints, group=ARM_GROUP):
        req = MotionPlanRequest()
        req.group_name = group
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self._p("vel_scale")
        req.max_acceleration_scaling_factor = self._p("vel_scale")
        req.goal_constraints.append(constraints)
        g = MoveGroup.Goal()
        g.request = req
        g.planning_options.planning_scene_diff.is_diff = True
        g.planning_options.planning_scene_diff.robot_state.is_diff = True
        g.planning_options.plan_only = False
        if not self._move.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("move_group not available"); return False
        gh = self._wait(self._move.send_goal_async(g))
        if gh is None or not gh.accepted:
            self.get_logger().error("move_group rejected goal"); return False
        res = self._wait(gh.get_result_async())
        if res is None:
            self.get_logger().error("move_group timed out"); return False
        code = res.result.error_code.val
        if code != 1:
            self.get_logger().warn(f"move failed: status={res.status} MoveItErrorCode={code}")
        return code == 1

    def _execute(self, gh):
        try:
            return self._run(gh)
        finally:
            self._busy = False

    def _run(self, gh):
        req = gh.request
        label = req.object_label
        place = req.place_pose
        result = PickObject.Result()

        def phase(name):
            fb = PickObject.Feedback(); fb.phase = name
            gh.publish_feedback(fb)
            self.get_logger().info(f"[{label}] {name}")

        def fail(msg):
            self.get_logger().error(f"[{label}] {msg}")
            result.success = False; result.message = msg
            gh.abort()
            return result

        hover = self._p("hover_height"); gz = self._p("grasp_z_offset")
        lift  = self._p("lift_height");  ph = self._p("place_hover")
        op    = self._p("gripper_open_rad"); grip = self._p("gripper_grip_rad")

        phase("DETECTING")
        obj = self._objects.get(label)
        if obj is None:
            return fail(f"'{label}' not currently detected")
        o = obj.pose.pose.position
        self.get_logger().info(f"object at ({o.x:.3f}, {o.y:.3f}, {o.z:.3f})")

        # Grasp point = tag position + tunable offset. The tag is on the box's
        # FRONT FACE, so the body to grab sits behind it (deeper in +x).
        gx = o.x + self._p("grasp_dx")
        gy = o.y + self._p("grasp_dy")
        gzp = o.z + self._p("grasp_dz")
        self.get_logger().info(f"grasp point ({gx:.3f}, {gy:.3f}, {gzp:.3f})")

        phase("APPROACHING")
        if not self._move_arm(gx, gy, gzp + hover): return fail("pre-grasp move failed")
        if not self._move_gripper(op):              return fail("gripper open failed")

        phase("GRASPING")
        if not self._move_arm(gx, gy, gzp):         return fail("descend-to-grasp failed")
        if not self._move_gripper(grip):            return fail("gripper close failed")
        if not self._move_arm(gx, gy, gzp + lift):  return fail("lift failed")
        
        phase("PLACING")
        if not self._move_arm(place.position.x, place.position.y, place.position.z + ph):
            return fail("pre-place move failed")
        if not self._move_arm(place.position.x, place.position.y, place.position.z):
            return fail("place descend failed")
        if not self._move_gripper(op):                return fail("release failed")
        self._move_arm(place.position.x, place.position.y, place.position.z + ph)

        result.success = True; result.message = f"Placed '{label}'."
        gh.succeed()
        self.get_logger().info(f"[{label}] DONE")
        return result


def main():
    rclpy.init()
    node = PickPlaceServer()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
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
