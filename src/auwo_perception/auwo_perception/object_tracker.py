#!/usr/bin/env python3
"""
object_tracker.py

Subscribes to AprilTag detections and converts them to a WorldModel
message expressed in the arm planning frame (arm_base_link).

Each AprilTag ID maps to a named object label defined in parameters.
The 3D pose comes from the TF transform that apriltag_ros publishes
for each detected tag.
"""
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
from auwo_interfaces.msg import DetectedObject, WorldModel


class ObjectTrackerNode(Node):

    def __init__(self):
        super().__init__('object_tracker')

        # Parameters
        self.declare_parameter('target_frame', 'arm_base_link')
        self.declare_parameter('tag_labels', ['0:object'])

        self.target_frame = self.get_parameter('target_frame').value

        # Build tag_id → label map from parameter list ["0:red_cube", "1:blue_cube"]
        self.tag_labels: dict[int, str] = {}
        for entry in self.get_parameter('tag_labels').value:
            try:
                tag_id_str, label = entry.split(':', 1)
                self.tag_labels[int(tag_id_str)] = label
            except ValueError:
                self.get_logger().warn(f'Bad tag_labels entry: {entry}')

        self.get_logger().info(
            f'Tracking tags: {self.tag_labels} → frame: {self.target_frame}')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers / Publishers
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self._detection_callback,
            10
        )
        self.world_model_pub = self.create_publisher(
            WorldModel,
            '/auwo/world_model',
            10
        )

        self.get_logger().info('Object tracker ready.')

    def _detection_callback(self, msg: AprilTagDetectionArray):
        world_model = WorldModel()
        world_model.header.stamp = self.get_clock().now().to_msg()
        world_model.header.frame_id = self.target_frame

        for detection in msg.detections:
            tag_id = detection.id
            label = self.tag_labels.get(tag_id, f'tag_{tag_id}')

            # apriltag_ros publishes TF for each detected tag as tag36h11:ID
            tag_frame = f'tag36h11:{tag_id}'

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    tag_frame,
                    Time(),                # latest available
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )

                obj = DetectedObject()
                obj.header.stamp = self.get_clock().now().to_msg()
                obj.header.frame_id = self.target_frame
                obj.label = label
                obj.confidence = float(detection.decision_margin) / 100.0
                obj.tracking_id = tag_id

                pose = PoseStamped()
                pose.header = obj.header
                t = transform.transform.translation
                r = transform.transform.rotation
                pose.pose.position.x = t.x
                pose.pose.position.y = t.y
                pose.pose.position.z = t.z
                pose.pose.orientation = r
                obj.pose = pose

                world_model.objects.append(obj)

                self.get_logger().info(
                    f'[{label}] pos=({t.x:.3f}, {t.y:.3f}, {t.z:.3f})',
                    throttle_duration_sec=1.0
                )

            except (LookupException, ConnectivityException,
                    ExtrapolationException) as e:
                self.get_logger().warn(
                    f'TF lookup failed for tag {tag_id} ({tag_frame}): {e}',
                    throttle_duration_sec=2.0
                )

        if world_model.objects:
            self.world_model_pub.publish(world_model)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()