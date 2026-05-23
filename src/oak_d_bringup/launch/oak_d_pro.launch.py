import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('oak_d_bringup'),
        'config', 'oak_d_pro.yaml'
    )

    return LaunchDescription([
        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            name='oak_d_pro',
            namespace='oak',
            parameters=[params_file],
            output='screen',
        ),
    ])
