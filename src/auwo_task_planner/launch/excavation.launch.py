from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = 'auwo_task_planner'
    common = dict(package=pkg, output='screen', emulate_tty=True)
    return LaunchDescription([
        Node(executable='sim_target_bridge',   name='sim_target_bridge',   **common),
        Node(executable='situational_context', name='situational_context', **common),
        Node(executable='estop',               name='estop',               **common),
        Node(executable='excavation_server',   name='excavation_server',   **common),
        Node(executable='planner',             name='planner',             **common),
    ])
