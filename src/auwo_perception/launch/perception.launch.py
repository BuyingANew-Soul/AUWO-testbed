import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('auwo_perception'),
        'config', 'apriltag.yaml'
    )

    relay_node = Node(
        package='auwo_perception',
        executable='image_relay',
        name='image_relay',
        output='screen',
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        parameters=[config],
        remappings=[
            # image_transport derives camera_info from image namespace automatically
            ('image_rect',  '/oak/rgb/image_relay'),
            ('camera_info', '/oak/rgb/camera_info'),
        ],
        output='screen',
    )

    tracker_node = Node(
        package='auwo_perception',
        executable='object_tracker',
        name='object_tracker',
        output='screen',
        parameters=[{
            'tag_labels':   ['0:red_cube', '1:blue_cube', '2:green_cube'],
            'target_frame': 'arm_base_link',
        }],
    )

    return LaunchDescription([
        relay_node,
        apriltag_node,
        tracker_node,
    ])