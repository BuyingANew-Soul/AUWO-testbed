from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    db_path = LaunchConfiguration("db_path")

    parameters = [{
        "use_sim_time": False,
        "frame_id": "base_link",
        "odom_frame_id": "odom",
        "map_frame_id": "map",
        "publish_tf": True,
        "subscribe_depth": True,
        "subscribe_rgb": True,
        "subscribe_scan": False,
        "subscribe_scan_cloud": False,
        "subscribe_rgbd": False,
        "subscribe_imu": True,
        "rgb/image_transport": "compressed",
        "depth/image_transport": "compressedDepth",
        "approx_sync": True,
        "sync_queue_size": 20,
        "topic_queue_size": 20,
        "qos": 2,
        "wait_imu_to_init": True,
        "wait_for_transform": 0.5,
        "database_path": db_path,
        "Mem/IncrementalMemory": "false",
        "Mem/InitWMWithAllNodes": "true",
        "RGBD/AngularUpdate": "0.05",
        "RGBD/LinearUpdate": "0.05",
        "Rtabmap/DetectionRate": "1.0",
        "Grid/FromDepth": "true",
        "Grid/3D": "false",
        "Grid/RayTracing": "true",
        "Grid/CellSize": "0.05",
        "Grid/RangeMax": "5.0",
        "Grid/MaxObstacleHeight": "2.0",
        "Grid/MinObstacleHeight": "0.1",
        "Reg/Strategy": "1",
        "Reg/Force3DoF": "true",
        "Icp/VoxelSize": "0.05",
        "Icp/MaxCorrespondenceDistance": "0.1",
        "Icp/MaxTranslation": "0.2",
        "Vis/MaxDepth": "4.0",
        "Vis/MinInliers": "15",
    }]

    remappings = [
        ("rgb/image", "/d435i_camera/color/image_raw"),
        ("rgb/camera_info", "/d435i_camera/color/camera_info"),
        ("depth/image", "/d435i_camera/depth/image_rect_raw"),
        ("odom", "/merged_odom"),
        ("imu", "/rtabmap/imu"),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            "db_path",
            default_value="~/.ros/rtabmap_mapping_depth_only.db",
            description="Path to RTAB-Map database",
        ),
        DeclareLaunchArgument(
            "rtabmap_viz",
            default_value="false",
            description="Launch RTAB-Map visualization",
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            namespace="rtabmap",
            output="screen",
            parameters=parameters,
            remappings=remappings,
        ),
        Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            output="screen",
            parameters=parameters,
            remappings=remappings,
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
        ),
    ])
