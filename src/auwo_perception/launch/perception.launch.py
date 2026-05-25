import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('auwo_perception'),
        'config', 'apriltag.yaml'
    )

    # AprilTag detector node — runs on laptop, subscribes to camera stream from Pi
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        parameters=[config],
        remappings=[
            # apriltag_ros expects image_rect — we remap to raw (OAK-D has low distortion)
            ('image_rect', '/oak/oak_d_pro/rgb/image_raw'),
            ('camera_info',  '/oak/oak_d_pro/rgb/camera_info'),
        ],
        output='screen',
    )

    # Object tracker node — converts tag detections to WorldModel
    tracker_node = Node(
        package='auwo_perception',
        executable='object_tracker',
        name='object_tracker',
        output='screen',
        parameters=[{
            # Map tag ID → object label
            'tag_labels': ['0:red_cube', '1:blue_cube', '2:green_cube'],
            # Frame to express object poses in (arm planning frame)
            'target_frame': 'arm_base_link',
        }],
    )

    return LaunchDescription([
        apriltag_node,
        tracker_node,
    ])