cmake_minimum_required(VERSION 3.5)
project(uuv_manipulators_kinematics)

find_package(ament_cmake REQUIRED)

# If you later port C++/Gazebo parts, add the needed deps, e.g.:
# find_package(gazebo_ros REQUIRED)
# find_package(orocos_kdl REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)

# Install Python kinematics service node
install(
  PROGRAMS
    scripts/kinematics_service.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
