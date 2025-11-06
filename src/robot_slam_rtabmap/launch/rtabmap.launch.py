from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    robot_slam_dir = get_package_share_directory('robot_slam_rtabmap')
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    
    # RTABMap parameters
    # NOTE: Parameters with capital letters (RTAB-Map specific) must be strings!
    parameters = [{
        'use_sim_time': use_sim_time,
        
        # Frame IDs
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'publish_tf': True,
        
        # Subscriptions
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': True,
        'subscribe_scan_cloud': False,
        'subscribe_rgbd': False,
        
        # Synchronization
        'approx_sync': True,
        'sync_queue_size': 30,  # Changed from queue_size
        'qos': 2,
        
        # Wait for transforms
        'wait_for_transform': 0.5,
        
        # Database
        'database_path': '~/.ros/rtabmap_leo.db',
        
        # SLAM vs Localization (native boolean OK)
        'Mem/IncrementalMemory': 'true',        # String!
        'Mem/InitWMWithAllNodes': 'false',      # String!
        
        # Basic SLAM parameters (native types OK for these)
        'RGBD/AngularUpdate': '0.05',           # String!
        'RGBD/LinearUpdate': '0.05',            # String!
        'Rtabmap/DetectionRate': '1.0',         # String!
        
        # Grid mapping (MUST be strings)
        'Grid/FromDepth': 'true',               # String!
        'Grid/3D': 1,                      # String! This was the error
        'Grid/RayTracing': 'true',              # String!
        'Grid/CellSize': '0.05',                # String!
        'Grid/RangeMax': '5.0',                 # String!
        'Grid/MaxObstacleHeight': '2.0',        # String!
        'Grid/MinObstacleHeight': '0.1',        # String!
        
        # Optimization (strings)
        'Reg/Strategy': '1',                    # String! 1=ICP
        'Reg/Force3DoF': 'true',                # String!
        
        # ICP parameters
        'Icp/VoxelSize': '0.05',               # String!
        'Icp/MaxCorrespondenceDistance': '0.1', # String!
        'Icp/MaxTranslation': '0.2',           # String!
        
        # Visual parameters
        'Vis/MaxDepth': '4.0',                 # String!
        'Vis/MinInliers': '15',                # String!
    }]
    
    # Topic remappings
    remappings = [
        ('rgb/image', '/d435i_camera/color/image_raw'),
        ('rgb/camera_info', '/d435i_camera/color/camera_info'),
        ('depth/image', '/d435i_camera/depth/image_rect_raw'),
        ('scan', '/scan'),
        ('odom', '/merged_odom'),
    ]
    
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
        
        # RTABMap SLAM Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=[
                '--delete_db_on_start',
            ],
            namespace='rtabmap',
        ),
        
        # RTABMap Visualization (optional)
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            condition=launch.conditions.IfCondition(
                LaunchConfiguration('rtabmap_viz')
            )
        ),
        
    ])