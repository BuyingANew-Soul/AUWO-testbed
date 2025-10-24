### AUWO-testbed

### **Package Structure

#### robot_autonomy_ws/
#### ├── src/
#### │   ├── robot_bringup/              #### Launch files
#### │   │   ├── launch/
#### │   │   │   ├── physical_robot.launch.py
#### │   │   │   ├── digital_twin.launch.py
#### │   │   │   ├── sensors.launch.py
#### │   │   │   ├── perception.launch.py
#### │   │   │   ├── navigation.launch.py
#### │   │   │   └── manipulation.launch.py
#### │   │   └── config/
#### │   │       ├── rtabmap.yaml
#### │   │       ├── nav2_params.yaml
#### │   │       └── moveit_config.yaml
#### │   │
#### │   ├── robot_description/          #### URDF/Xacro
#### │   │   ├── urdf/
#### │   │   │   ├── leo_base.urdf.xacro
#### │   │   │   ├── roarm_m2s.urdf.xacro    #### Current arm
#### │   │   │   ├── pincherx100.urdf.xacro  #### Future arm
#### │   │   │   └── robot.urdf.xacro        #### Combined
#### │   │   └── meshes/
#### │   │
#### │   ├── robot_perception/           #### Perception stack
#### │   │   ├── src/
#### │   │   │   ├── object_detector_3d.py
#### │   │   │   ├── terrain_analyzer.py
#### │   │   │   ├── scene_understanding.py
#### │   │   │   └── pointcloud_processor.cpp
#### │   │   └── config/
#### │   │       ├── yolo_config.yaml
#### │   │       └── detection_classes.yaml
#### │   │
#### │   ├── robot_planning/             #### Mission planning
#### │   │   ├── src/
#### │   │   │   ├── mission_planner.py      #### Main state machine
#### │   │   │   ├── navigation_interface.py
#### │   │   │   ├── manipulation_interface.py
#### │   │   │   └── behavior_trees/
#### │   │   │       └── pickup_mission.xml
#### │   │   └── include/
#### │   │
#### │   ├── robot_manipulation/         #### Arm control
#### │   │   ├── src/
#### │   │   │   ├── arm_controller.py
#### │   │   │   ├── gripper_controller.py
#### │   │   │   ├── grasp_planner.py
#### │   │   │   └── moveit_interface.py
#### │   │   ├── config/
#### │   │   │   └── moveit/                 #### MoveIt2 configs
#### │   │   └── launch/
#### │   │
#### │   ├── robot_navigation/           #### Nav2 configs
#### │   │   ├── config/
#### │   │   │   ├── costmap_common.yaml
#### │   │   │   ├── local_costmap.yaml
#### │   │   │   ├── global_costmap.yaml
#### │    │   │   └── planner_params.yaml
#### │   │   └── maps/                       #### Saved maps
#### │   │
#### │   ├── robot_interfaces/           #### Custom messages/actions
#### │   │   ├── msg/
#### │   │   │   ├── DetectedObject3D.msg
#### │   │   │   ├── GraspCandidate.msg
#### │   │   │   ├── TerrainInfo.msg
#### │   │   │   └── RobotStatus.msg
#### │   │   ├── action/
#### │   │   │   ├── PickupMission.action
#### │   │   │   ├── NavigateToObject.action
#### │   │   │   └── ExecuteManipulation.action
#### │   │   └── srv/
#### │   │       ├── DetectObjects.srv
#### │   │       └── PlanGrasp.srv
#### │   │
#### │   ├── robot_safety/               #### Safety monitoring
#### │   │   └── src/
#### │   │       ├── safety_monitor.py
#### │   │       └── collision_checker.py
#### │   │
#### │   ├── robot_digital_twin/         #### Simulation
#### │   │   ├── worlds/
#### │   │   ├── models/
#### │   │   └── launch/
#### │   │       └── gazebo_sim.launch.py
#### │   │
#### │   └── robot_telemetry/            #### Data logging & monitoring
#### │       └── src/
#### │           ├── data_logger.py
#### │           ├── ros2_bridge.py          #### For ROS2 → ROS1 if needed
#### │           └── web_interface.py
