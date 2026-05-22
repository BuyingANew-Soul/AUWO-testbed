import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('ugv_with_arm_camera_description')

    robot_description = ParameterValue(
        Command([
            'xacro ',
            os.path.join(pkg, 'urdf', 'robots', 'ugv02_with_arm.urdf.xacro')
        ]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gui', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 50.0,
            }],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('use_gui')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
