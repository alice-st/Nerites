cmake_minimum_required(VERSION 3.5)
project(oberon7_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes params robots urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE
)

ament_package()
