import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_pkg = get_package_share_directory("robot_navigation")

    use_rviz = LaunchConfiguration("use_rviz")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz on this machine (set false on robot)",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(nav_pkg, "config", "nav2_params_depth_only.yaml"),
            description="Path to Nav2 parameter file",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "use_sim_time": "false",
                "params_file": params_file,
                "autostart": "true",
                "use_collision_monitor": "true",
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                os.path.join(get_package_share_directory("nav2_bringup"), "rviz", "nav2_default_view.rviz"),
            ],
            output="screen",
            condition=IfCondition(use_rviz),
        ),
    ])
