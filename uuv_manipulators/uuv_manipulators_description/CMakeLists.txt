cmake_minimum_required(VERSION 3.5)
project(uuv_manipulators_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
)

ament_package()
