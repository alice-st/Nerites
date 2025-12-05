cmake_minimum_required(VERSION 3.5)
project(oberon4_control)

find_package(ament_cmake REQUIRED)

# Install launch files (ROS 2 style)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
)

ament_package()
