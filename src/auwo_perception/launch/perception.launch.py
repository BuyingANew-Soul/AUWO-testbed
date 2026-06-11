from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Camera frame now comes from the robot model (demo.launch.py).
    # No static TF bridge needed — just the tracker, outputting poses in base_link.
    tracker_node = Node(
        package='auwo_perception',
        executable='object_tracker',
        name='object_tracker',
        output='screen',
        parameters=[{
            'tag_labels':   ['0:red_cube', '1:blue_cube', '2:green_cube'],
            'target_frame': 'base_link',
        }],
    )
    return LaunchDescription([tracker_node])
