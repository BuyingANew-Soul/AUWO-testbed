# launch/navigation.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Packages
    nav_pkg = get_package_share_directory('robot_navigation')
    # slam_pkg = get_package_share_directory('robot_slam_rtabmap')

    # Launch arguments
    db_path = LaunchConfiguration('db_path')

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'db_path',
        #     default_value=os.path.expanduser('~/.ros/rtabmap_leo.db'),
        #     description='Full path to the RTAB-Map database'
        # ),

        # # 1. RTAB-Map in localization mode (your updated launch file)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(slam_pkg, 'launch', 'rtabmap.launch.py')
        #     ),
        #     launch_arguments={
        #         'use_sim_time': 'false',
        #         'localization': 'true',
        #         'delete_db_on_start': 'false',
        #         'db_path': db_path,
        #         'rtabmap_viz': 'false',
        #     }.items()
        # ),

        # 2. Nav2 stack (no AMCL, no map_server → RTAB-Map provides /map)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'params_file': os.path.join(nav_pkg, 'config', 'nav2_params_original.yaml'),
                'autostart': 'true',
                # Very simple BT: just go to pose with basic recovery
                'default_nav_to_pose_bt_xml': os.path.join(nav_pkg, 'config', 'simple_navigate_to_pose.xml'),
            }.items()
        ),

        # 3. RViz (optional but super helpful)
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')],
            output='screen'
        ),
    ])