cmake_minimum_required(VERSION 3.8)
project(reach_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY xacro meshes rviz config DESTINATION share/reach_description)

ament_package()
