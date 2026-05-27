"""
Relays compressed RGB image → raw (resized) and camera_info to locations
that image_transport expects (same namespace as the image topic).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge
import cv2


TARGET_WIDTH  = 848   # resize to keep apriltag detection reliable
TARGET_HEIGHT = 480


class ImageRelayNode(Node):

    def __init__(self):
        super().__init__('image_relay')
        self.bridge    = CvBridge()
        self.received  = 0
        self.published = 0

        # ── Image: compressed in, resized raw out ──────────────────────────
        self.img_sub = self.create_subscription(
            CompressedImage,
            '/oak/oak_d_pro/rgb/image_raw/compressed',
            self._img_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=2
            )
        )
        self.img_pub = self.create_publisher(
            Image, '/oak/rgb/image_relay', 2)

        # ── Camera info: forward to namespace image_transport expects ───────
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/oak/oak_d_pro/rgb/camera_info',
            self._info_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=2
            )
        )
        self.info_pub = self.create_publisher(
            CameraInfo, '/oak/rgb/camera_info', 2)

        self.scale_x = TARGET_WIDTH  / 1920.0
        self.scale_y = TARGET_HEIGHT / 1080.0

        self.create_timer(5.0, self._log_stats)
        self.get_logger().info(
            f'Image relay ready — resizing to {TARGET_WIDTH}×{TARGET_HEIGHT}')

    def _img_callback(self, msg: CompressedImage):
        self.received += 1
        try:
            img = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            img = cv2.resize(
                img, (TARGET_WIDTH, TARGET_HEIGHT),
                interpolation=cv2.INTER_LINEAR)
            out = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            out.header = msg.header
            self.img_pub.publish(out)
            self.published += 1
        except Exception as e:
            self.get_logger().warn(str(e), throttle_duration_sec=2.0)

    def _info_callback(self, msg: CameraInfo):
        """Forward camera_info, scaling intrinsics to match resized image."""
        scaled              = CameraInfo()
        scaled.header       = msg.header
        scaled.width        = TARGET_WIDTH
        scaled.height       = TARGET_HEIGHT
        scaled.distortion_model = msg.distortion_model
        scaled.d            = msg.d

        # Scale K matrix: fx, cx scale by x; fy, cy scale by y
        k = list(msg.k)
        k[0] *= self.scale_x   # fx
        k[2] *= self.scale_x   # cx
        k[4] *= self.scale_y   # fy
        k[5] *= self.scale_y   # cy
        scaled.k = k

        # P matrix same scaling
        p = list(msg.p)
        p[0] *= self.scale_x
        p[2] *= self.scale_x
        p[5] *= self.scale_y
        p[6] *= self.scale_y
        scaled.p = p

        scaled.r = msg.r
        self.info_pub.publish(scaled)

    def _log_stats(self):
        self.get_logger().info(
            f'Stats — received: {self.received}, published: {self.published}')
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()