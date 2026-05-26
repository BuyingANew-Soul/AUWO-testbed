#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class ImageRelayNode(Node):

    def __init__(self):
        super().__init__('image_relay')

        self.declare_parameter(
            'input_topic', '/oak/oak_d_pro/rgb/image_raw/compressed')
        self.declare_parameter(
            'output_topic', '/oak/rgb/image_relay')

        input_topic  = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.bridge   = CvBridge()
        self.received = 0
        self.published = 0

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5          # increased from 1 to 5
        )

        self.sub = self.create_subscription(
            CompressedImage,
            input_topic,
            self._callback,
            best_effort_qos
        )
        self.pub = self.create_publisher(Image, output_topic, 5)

        # Log stats every 5 seconds
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(f'Relay: {input_topic} → {output_topic}')

    def _callback(self, msg: CompressedImage):
        self.received += 1
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            out.header = msg.header
            self.pub.publish(out)
            self.published += 1
        except Exception as e:
            self.get_logger().warn(
                f'Decode failed: {e}', throttle_duration_sec=2.0)

    def _log_stats(self):
        self.get_logger().info(
            f'Relay stats — received: {self.received}, '
            f'published: {self.published}')
        self.received  = 0
        self.published = 0


def main(args=None):
    rclpy.init(args=args)
    node = ImageRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()