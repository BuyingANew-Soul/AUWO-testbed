#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped for buffer.transform)


class SimTargetBridge(Node):
    def __init__(self):
        super().__init__("sim_target_bridge")
        # map sim prim names -> output topics: dig cone and dump cone
        self.declare_parameter("child_frames", ["Cone", "DumpCone"])
        self.declare_parameter("out_topics", ["/excavation/target", "/excavation/dump_target"])
        self.declare_parameter("in_topic", "/sim/target_tf")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("base_frame", "base_link")

        frames = list(self.get_parameter("child_frames").value)
        topics = list(self.get_parameter("out_topics").value)
        self._world = self.get_parameter("world_frame").value
        self._base = self.get_parameter("base_frame").value
        self._map = {f: t for f, t in zip(frames, topics)}
        self._pubs = {f: self.create_publisher(PointStamped, t, 10) for f, t in self._map.items()}

        self._buf = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buf, self)
        self.create_subscription(TFMessage, self.get_parameter("in_topic").value, self._cb, 10)
        self.get_logger().info(f"Bridging {self._map} from {self._world} -> {self._base} via TF")

    def _cb(self, msg):
        for tf in msg.transforms:
            f = tf.child_frame_id
            if f not in self._map:
                continue
            p = PointStamped()
            p.header.frame_id = self._world
            p.header.stamp = Time().to_msg()
            p.point.x = tf.transform.translation.x
            p.point.y = tf.transform.translation.y
            p.point.z = tf.transform.translation.z
            try:
                out = self._buf.transform(p, self._base, timeout=Duration(seconds=0.2))
            except Exception as e:
                self.get_logger().warn(f"TF {self._world}->{self._base} for '{f}' unavailable: {e}",
                                       throttle_duration_sec=2.0)
                continue
            self._pubs[f].publish(out)


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