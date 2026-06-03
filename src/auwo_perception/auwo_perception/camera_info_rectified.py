"""
Republish camera_info with zero distortion to match the rectified image.
After image_proc rectification, the image is undistorted — apriltag must
receive matching zero-distortion camera_info or depth estimation is wrong.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraInfoRectifiedNode(Node):
    def __init__(self):
        super().__init__('camera_info_rectified')
        self.sub = self.create_subscription(
            CameraInfo,
            '/oak/oak_d_pro/rgb/camera_info',
            self._callback,
            10,
        )
        self.pub = self.create_publisher(CameraInfo, '/oak/rgb/camera_info', 10)
        self.get_logger().info('Camera info rectified publisher ready.')

    def _callback(self, msg: CameraInfo):
        out = CameraInfo()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width

        # Rectified image uses P matrix as intrinsics, zero distortion
        out.distortion_model = 'plumb_bob'
        out.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # K = upper-left 3×3 of P (Tx=0 for monocular)
        p = msg.p  # flat 12-element list
        out.k = [p[0], p[1], p[2],
                 p[4], p[5], p[6],
                 p[8], p[9], p[10]]

        out.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        out.p = list(msg.p)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoRectifiedNode()
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