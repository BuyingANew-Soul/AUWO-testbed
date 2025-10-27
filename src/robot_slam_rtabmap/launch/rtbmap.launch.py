from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    
    robot_slam_dir = get_package_share_directory('robot_slam_rtabmap')
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    parameters=[{
          'frame_id':'camera_link',
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False,
          'wait_imu_to_init':True}]

    remappings = [
                # RGB-D Camera remappings (D435i)
                ('rgb/image', '/d435i_camera/color/image_raw'),
                ('rgb/camera_info', '/d435i_camera/color/camera_info'),
                ('depth/image', '/d435i_camera/depth/image_rect_raw'),
                
                # Lidar remapping
                ('scan', '/scan'),
                
                # Odometry from Leo's merged odometry
                ('odom', '/merged_odom'),]
    
    config_file = os.path.join(
        robot_slam_dir,
        'config',
        'rtbmap_config.yaml'
    )
    
    return LaunchDescription([
        
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Launch in localization mode'
        ),
        
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='Launch RTABMap visualization'
        ),
        
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz'
        ),
        
        # RTABMap SLAM Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=[
                config_file,
                {
                    'use_sim_time': use_sim_time,
                    'localization': localization,
                }
            ],
            remappings= remappings,
            arguments=[
                '--delete_db_on_start',  # Start fresh (remove this to keep map)
                '--udebug'               # Enable debug output
            ],
            namespace=''
        ),
        
        # RTABMap Visualization (optional)
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=[
                config_file,
                {
                    'use_sim_time': use_sim_time,
                }
            ],
            remappings= remappings,
            condition=launch.conditions.IfCondition(
                LaunchConfiguration('rtabmap_viz')
            )
        ),
        
        # RViz (optional)
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(robot_slam_dir, 'rviz', 'rtabmap_leo.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=launch.conditions.IfCondition(
                LaunchConfiguration('rviz')
            )
        ),
        
    ])