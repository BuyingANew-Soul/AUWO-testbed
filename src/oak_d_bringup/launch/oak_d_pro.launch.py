import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('oak_d_bringup'),
        'config', 'oak_d_pro.yaml'
    )

    camera_node = Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='oak_d_pro',
            namespace='oak',
            parameters=[params_file],
            output='screen',
    )

    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rgb_rectify',
        remappings=[
            ('image',       '/oak/oak_d_pro/rgb/image_raw'),
            ('camera_info', '/oak/oak_d_pro/rgb/camera_info'),
            ('image_rect',  '/oak/rgb/image_rect'),
        ],
        output='screen',
    )

    return LaunchDescription([camera_node, rectify_node])