from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Detection now happens on the Pi. This only runs the tracker, which
    # combines the tag TF (from Pi's apriltag) with the robot TF (from
    # robot_state_publisher) and publishes /auwo/world_model.
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
    return LaunchDescription([tracker_node])