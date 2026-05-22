cmake_minimum_required(VERSION 3.8)
project(ugv_with_arm_camera_description)

find_package(ament_cmake REQUIRED)

# Install all directories so launch files can find them at runtime
install(DIRECTORY
  urdf
  launch
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()