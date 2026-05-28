import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('auwo_perception'),
        'config', 'apriltag.yaml'
    )

    # rectify_node = Node(
    #     package='image_proc',
    #     executable='rectify_node',
    #     name='image_rectify',
    #     remappings=[
    #         # Explicitly remap the compressed sub-topic
    #         ('image/compressed',  '/oak/oak_d_pro/rgb/image_raw/compressed'),
    #         ('camera_info',       '/oak/oak_d_pro/rgb/camera_info'),
    #         ('image_rect',        '/oak/rgb/image_rect'),
    #     ],
    #     output='screen',
    # )

    # camera_info_relay = Node(
    #     package='topic_tools',
    #     executable='relay',
    #     name='camera_info_relay',
    #     arguments=[
    #         '/oak/oak_d_pro/rgb/camera_info',
    #         '/oak/rgb/camera_info',
    #     ],
    #     output='screen',
    # )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        parameters=[config],
        remappings=[
            ('image_rect',  '/oak/rgb/image_rect'),
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

        apriltag_node,
        tracker_node,
    ])