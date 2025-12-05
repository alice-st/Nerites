cmake_minimum_required(VERSION 3.5)
project(eca_a9_control)

find_package(ament_cmake REQUIRED)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" 
  EXCLUDE
)

ament_package()
