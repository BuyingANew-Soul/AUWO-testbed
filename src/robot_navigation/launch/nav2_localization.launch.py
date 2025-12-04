"""
Launch file for NAVIGATION MODE (with localization)
Use this when you have a pre-built map and want autonomous navigation
RTABMAP runs in localization mode
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # # Launch arguments
    # db_path = LaunchConfiguration('db_path')
    # map_yaml = LaunchConfiguration('map_yaml')
    
    return LaunchDescription([
        
        # DeclareLaunchArgument(
        #     'db_path',
        #     default_value='~/.ros/rtabmap_mapping.db',
        #     description='Path to pre-built RTABMAP database'
        # ),
        
        # DeclareLaunchArgument(
        #     'map_yaml',
        #     default_value='',
        #     description='Optional: Path to saved 2D map yaml. If empty, RTABMAP publishes map from database'
        # ),
        
        # # Launch sensors
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('robot_bringup'),
        #             'launch',
        #             'sensors.launch.py'
        #         ])
        #     ])
        # ),
        
        # Launch RTABMAP in LOCALIZATION mode
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('robot_slam_rtabmap'),
        #             'launch',
        #             'rtabmap.launch.py'
        #         ])
        #     ]),
        #     launch_arguments={
        #         'db_path': db_path,
        #         'localization': 'true',  # LOCALIZATION mode
        #         'rtabmap_viz': 'false'
        #     }.items()
        # ),
        
        # Launch Nav2 for navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_navigation'),
                    'launch',
                    'navigation.launch.py'
                ])
            ]),
            launch_arguments={
                'map': map_yaml,  # Optional: can be empty if using RTABMAP's /map
            }.items()
        ),
    ])