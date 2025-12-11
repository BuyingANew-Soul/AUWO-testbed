from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # --- Gazebo / Digital Twin (your existing setup) -----------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("excavator_gazebo"),
                "/launch/gazebo.launch.py",
            ]
        )
    )

    # --- RViz config -------------------------------------------------------
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare("excavator_description"),
            "config",
            "view.rviz",
        ]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="auwo_digital_twin_rviz",
        arguments=["-d", rviz_config],
        output="screen",
    )

    # --- Interactive Markers for all joints --------------------------------
    joint_imarkers = Node(
        package="excavator_interactive_rviz",
        executable="joint_imarkers.py", 
        name="excavator_joint_imarkers",
        output="screen",
    )

    # --- Twin Router Node (unified /excavator_twin/* API) ------------------
    twin_router_node = Node(
        package="excavator_interactive_rviz",
        executable="twin_router_node.py",
        name="excavator_twin_router",
        output="screen",
        parameters=[
            {"default_mode": "simulation"},  # simulation / physical / dual / disabled
            # Digital twin (Gazebo) topics
            {"sim_command_topic": "/arm_position_controller/commands"},
            {"sim_state_topic": "/joint_states"},
            # Physical twin (Novatron) topics - placeholders for later
            {"physical_command_topic": "/physical_twin/commands"},
            {"physical_state_topic": "/physical_twin/state"},
        ],
    )

    return LaunchDescription(
        [
            gazebo_launch,
            rviz,
            joint_imarkers,
            twin_router_node,
        ]
    )

