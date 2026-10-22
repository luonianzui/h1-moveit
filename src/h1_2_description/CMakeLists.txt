cmake_minimum_required(VERSION 3.16)
project(h1_2_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes meshes_low urdf vendor models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
