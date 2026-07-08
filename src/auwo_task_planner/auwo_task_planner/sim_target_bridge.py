#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PointStamped, TransformStamped
import tf2_ros


class SimTargetBridge(Node):
    def __init__(self):
        super().__init__("sim_target_bridge")
        self.declare_parameter("child_frames", ["Cone", "DumpCone"])
        self.declare_parameter("out_topics", ["/excavation/target", "/excavation/dump_target"])
        self.declare_parameter("in_topic", "/sim/target_tf")
        self.declare_parameter("base_frame", "base_link")

        frames = list(self.get_parameter("child_frames").value)
        topics = list(self.get_parameter("out_topics").value)
        self._base = self.get_parameter("base_frame").value
        self._map = {f: t for f, t in zip(frames, topics)}
        self._pubs = {f: self.create_publisher(PointStamped, t, 10) for f, t in self._map.items()}

        # private buffer fed ONLY from /sim/target_tf — never touches global /tf
        self._buf = tf2_ros.Buffer()
        self.create_subscription(TFMessage, self.get_parameter("in_topic").value, self._cb, 10)
        self.get_logger().info(f"Bridging {self._map} -> {self._base} via private tf2 buffer")

    def _cb(self, msg):
        # load every Isaac transform into the private buffer
        for tf in msg.transforms:
            t = TransformStamped()
            t.header.frame_id = tf.header.frame_id
            t.header.stamp = Time().to_msg()   # zero stamp -> treated as latest
            t.child_frame_id = tf.child_frame_id
            t.transform = tf.transform
            self._buf.set_transform(t, "sim_bridge")

        # now resolve each marker relative to base_link
        for frame, pub in ((f, self._pubs[f]) for f in self._map):
            try:
                tr = self._buf.lookup_transform(self._base, frame, Time())
            except Exception as e:
                self.get_logger().warn(f"lookup {self._base}<-{frame} failed: {e}",
                                       throttle_duration_sec=2.0)
                continue
            ps = PointStamped()
            ps.header.frame_id = self._base
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.point.x = tr.transform.translation.x
            ps.point.y = tr.transform.translation.y
            ps.point.z = tr.transform.translation.z
            pub.publish(ps)


def main():
    rclpy.init()
    node = SimTargetBridge()
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