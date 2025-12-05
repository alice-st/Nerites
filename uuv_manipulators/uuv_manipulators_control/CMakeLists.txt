cmake_minimum_required(VERSION 3.5)
project(uuv_manipulators_control)

find_package(ament_cmake REQUIRED)

# Install Python nodes (ROS 2 replacement for catkin_install_python)
install(
  PROGRAMS
    scripts/set_joint_config.py
    scripts/jt_cartesian_controller.py
    scripts/gripper_controller.py
    scripts/joint_position_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
