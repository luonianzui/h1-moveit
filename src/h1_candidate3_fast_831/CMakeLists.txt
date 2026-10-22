cmake_minimum_required(VERSION 3.8)
project(h1_candidate3_fast_831)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})

ament_package()
