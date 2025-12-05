cmake_minimum_required(VERSION 3.5)
project(uuv_manipulators_msgs)

# Default to C99 / C++14
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message/service dependencies
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EndPointState.msg"
  "msg/EndeffectorCommand.msg"
  "msg/EndeffectorState.msg"
  "msg/ArmConfigCommand.msg"
  "srv/SolveIK.srv"
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs
    builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
