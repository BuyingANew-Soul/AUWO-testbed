#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from auwo_interfaces.msg import WorldModel

ARM_GROUP      = "arm"
EE_LINK        = "hand_tcp"
PLANNING_FRAME = "base_link"
HOVER_Z        = 0.08     # metres above the object for the pre-grasp
POS_TOL        = 0.01     # metres, radius of the goal tolerance sphere


class GraspNode(Node):
    def __init__(self):
        super().__init__("grasp_node")
        self._objects = {}
        self.create_subscription(WorldModel, "/auwo/world_model", self._wm_cb, 10)
        self._move = ActionClient(self, MoveGroup, "/move_action")
        self.create_service(Trigger, "grasp/move_to_pregrasp", self._on_trigger)
        self.get_logger().info("Grasp node ready. Call /grasp/move_to_pregrasp to hover above a detected object.")

    def _wm_cb(self, msg):
        objs = None
        for field in ("objects", "detections", "detected_objects", "items"):
            if hasattr(msg, field):
                objs = getattr(msg, field)
                break
        if objs is None:
            return
        self._objects = {o.label: o for o in objs}

    def _on_trigger(self, request, response):
        if not self._objects:
            response.success = False
            response.message = "No objects detected yet."
            return response
        label, obj = next(iter(self._objects.items()))
        p = obj.pose.pose.position
        target = (p.x, p.y, p.z + HOVER_Z)
        self.get_logger().info(
            f"Pre-grasp above '{label}': object=({p.x:.3f},{p.y:.3f},{p.z:.3f}) "
            f"-> target=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f})")
        if not self._move.wait_for_server(timeout_sec=3.0):
            response.success = False
            response.message = "move_group action server not available."
            return response
        self._move.send_goal_async(self._position_goal(target)).add_done_callback(self._goal_resp_cb)
        response.success = True
        response.message = f"Motion to pre-grasp above '{label}' started."
        return response

    def _position_goal(self, xyz):
        req = MotionPlanRequest()
        req.group_name = ARM_GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2

        pc = PositionConstraint()
        pc.header.frame_id = PLANNING_FRAME
        pc.link_name = EE_LINK
        pc.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [POS_TOL]
        region = BoundingVolume()
        region.primitives.append(sphere)
        sp = Pose()
        sp.position.x, sp.position.y, sp.position.z = xyz
        sp.orientation.w = 1.0
        region.primitive_poses.append(sp)
        pc.constraint_region = region

        c = Constraints()
        c.position_constraints.append(pc)
        req.goal_constraints.append(c)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False
        return goal

    def _goal_resp_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error("move_group rejected the goal.")
            return
        gh.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        code = future.result().result.error_code.val
        if code == 1:
            self.get_logger().info("Pre-grasp reached.")
        else:
            self.get_logger().warn(f"Move failed (MoveItErrorCode={code}). -7 = no IK / out of reach.")


def main():
    rclpy.init()
    node = GraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()