### AUWO-testbed

### **Workspace Structure

#### AUWO-testbed/
#### ├── src/
#### │   ├── robot_bringup/              #### Launch files
#### │   │   ├── launch/
#### │   │   │   ├── full_system.launch.py
#### │   │   │   ├── digital_twin.launch.py (later)
#### │   │   │   ├── sensors.launch.py
#### │   │   │   ├── mapping.launch.py
#### │   │   │   ├── navigation.launch.py
#### │   │   │   └── manipulation.launch.py (later)
#### │   │   │   ├── realsense_d435if.launch.py
#### │   │   │   ├── realsense_d435if_launch.xml
#### │   │   │   ├── rplidar_s3.launch.py
#### │   │   │   ├── rplidar_s3_launch.xml
#### │   │   │   ├── robot_state.launch.py
#### │   │   └── config/

#### │   │
#### │   ├── robot_description/          #### URDF/Xacro
#### │   │   ├── urdf/
#### │   │   │   ├── realsense_d435if.urdf.xacro    #### Current arm
#### │   │   │   ├── rplidar.urdf.xacro  #### Future arm
#### │   │   │   └── robot.urdf.xacro        #### Combined
#### │   │   └── meshes/

#### │   │
#### │   ├── robot_slam_rtabmap/             #### RTABMAP SLAM
#### │   │   ├── launch/
#### │   │   │   ├── rtabmap.launch.py
#### │   │   └── config/
#### │   │   │   ├── rtabmap_config.yaml

#### │   ├── robot_perception/           #### Perception stack

#### │   │
#### │   ├── robot_planning/             #### Mission planning

#### │   │
#### │   ├── robot_manipulation/         #### Arm control

#### │   │
#### │   ├── robot_navigation/           #### Nav2 configs

#### │   │
#### │   ├── robot_interfaces/           #### Custom messages/actions

#### │   │
#### │   ├── robot_safety/               #### Safety monitoring

#### │   │
#### │   ├── robot_digital_twin/         #### Simulation

#### │   │
#### │   └── robot_telemetry/            #### logging and monitoring

