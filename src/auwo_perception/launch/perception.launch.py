import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('auwo_perception'),
        'config', 'apriltag.yaml'
    )

    # ---------------------------------------------------------------------------
    # 1. republish:  compressed (network) → raw (local, in-memory only)
    #    This is the KEY node: subscribes to the cheap compressed topic over the
    #    network and republishes as raw locally on the laptop — zero extra
    #    network cost compared to subscribing to /image_raw directly.
    # ---------------------------------------------------------------------------
    # republish_node = Node(
    #     package='image_transport',
    #     executable='republish',
    #     name='image_republish',
    #     arguments=['compressed', 'raw',
    #             '--ros-args',
    #             '-r', 'in/compressed:=/oak/oak_d_pro/rgb/image_raw/compressed',
    #             '-r', 'out:=/oak/rgb/image_raw_local'],
    #     parameters=[{
    #         'in_transport':  'compressed',
    #         'out_transport': 'raw',
    #     }],
    #     output='screen',
    )

    # ---------------------------------------------------------------------------
    # 2. rectify:  uses the LOCAL raw topic — never touches the network raw feed
    # ---------------------------------------------------------------------------
    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='image_rectify',
        parameters=[{
            'image_transport': 'compressed',
        }],
        remappings=[
            ('image',       '/oak/oak_d_pro/rgb/image_raw'),
            ('camera_info', '/oak/oak_d_pro/rgb/camera_info'),
            ('image_rect',  '/oak/rgb/image_rect'),
        ],
        output='screen',
    )

    # ---------------------------------------------------------------------------
    # 3. camera_info relay:  republishes camera_info under the /oak/rgb namespace
    #    so apriltag_node can find both image_rect and camera_info in one place.
    #    (topic_tools relay is the cleanest way; no custom node needed)
    # ---------------------------------------------------------------------------
    camera_info_relay = Node(
        package='topic_tools',
        executable='relay',
        name='camera_info_relay',
        arguments=[
            '/oak/oak_d_pro/rgb/camera_info',
            '/oak/rgb/camera_info',
        ],
        output='screen',
    )

    # ---------------------------------------------------------------------------
    # 4. apriltag detector
    # ---------------------------------------------------------------------------
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        parameters=[config],
        remappings=[
            ('image_rect',   '/oak/rgb/image_rect'),
            ('camera_info',  '/oak/rgb/camera_info'),   # ← now has a source
        ],
        output='screen',
    )

    # ---------------------------------------------------------------------------
    # 5. object tracker
    # ---------------------------------------------------------------------------
    tracker_node = Node(
        package='auwo_perception',
        executable='object_tracker',
        name='object_tracker',
        output='screen',
        parameters=[{
            'tag_labels':   ['0:red_cube', '1:blue_cube', '2:green_cube'],
            'target_frame': 'arm_base_link',
        }],
    )

    return LaunchDescription([
        # republish_node,
        rectify_node,
        camera_info_relay,
        # apriltag_node,   # uncomment when ready
        # tracker_node,    # uncomment when ready
    ])