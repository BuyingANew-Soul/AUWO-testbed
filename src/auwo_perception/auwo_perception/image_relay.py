#!/usr/bin/env python3
"""
Subscribes to compressed camera images with BEST_EFFORT QoS,
decompresses and republishes as raw with RELIABLE QoS for apriltag_ros.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image
import cv2
import numpy as np


class ImageRelayNode(Node):

    def __init__(self):
        super().__init__('image_relay')

        self.declare_parameter(
            'input_topic', '/oak/oak_d_pro/rgb/image_raw/compressed')
        self.declare_parameter(
            'output_topic', '/oak/rgb/image_relay')

        input_topic  = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub = self.create_subscription(
            CompressedImage,
            input_topic,
            self._callback,
            best_effort_qos
        )
        self.pub = self.create_publisher(Image, output_topic, 1)

        self.get_logger().info(
            f'Relay: {input_topic} → {output_topic}')

    def _callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        img    = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn('Failed to decode compressed image', 
                                   throttle_duration_sec=5.0)
            return

        out             = Image()
        out.header      = msg.header
        out.height      = img.shape[0]
        out.width       = img.shape[1]
        out.encoding    = 'bgr8'
        out.is_bigendian = 0
        out.step        = img.shape[1] * 3
        out.data        = img.tobytes()
        self.pub.publish(out)


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