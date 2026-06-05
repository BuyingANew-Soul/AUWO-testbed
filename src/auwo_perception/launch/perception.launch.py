import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    cam_config = os.path.join(
        get_package_share_directory('oak_d_bringup'),
        'config', 'oak_d_pro.yaml'
    )
    tag_config = os.path.join(
        get_package_share_directory('auwo_perception'),
        'config', 'apriltag.yaml'
    )

    # Camera + rectify composed → intra-process zero-copy for the raw image
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='depthai_ros_driver',
                plugin='depthai_ros_driver::Camera',
                name='oak_d_pro',
                namespace='oak',
                parameters=[cam_config],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rgb_rectify',
                remappings=[
                    ('image',       '/oak/oak_d_pro/rgb/image_raw'),
                    ('camera_info', '/oak/oak_d_pro/rgb/camera_info'),
                    ('image_rect',  '/oak/rgb/image_rect'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    # apriltag runs standalone (no composable component in this build)
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        parameters=[tag_config],
        remappings=[
            ('image_rect',  '/oak/rgb/image_rect'),
            ('camera_info', '/oak/rgb/camera_info'),
        ],
        output='screen',
    )

    # camera_info to where apriltag's image_transport expects it
    camera_info_relay = Node(
        package='topic_tools',
        executable='relay',
        name='camera_info_relay',
        arguments=['/oak/oak_d_pro/rgb/camera_info', '/oak/rgb/camera_info'],
        output='screen',
    )

    return LaunchDescription([container, apriltag_node, camera_info_relay])