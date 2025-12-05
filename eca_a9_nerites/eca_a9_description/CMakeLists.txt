cmake_minimum_required(VERSION 3.5)
project(eca_a9_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY mesh robots urdf launch
  DESTINATION share/${PROJECT_NAME} 
  PATTERN "*~" 
  EXCLUDE
)

ament_package()
