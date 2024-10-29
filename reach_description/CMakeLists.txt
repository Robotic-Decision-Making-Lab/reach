cmake_minimum_required(VERSION 3.8)
project(reach_description)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY description meshes rviz config
    DESTINATION share/reach_description
)

ament_package()
