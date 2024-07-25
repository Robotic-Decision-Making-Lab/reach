cmake_minimum_required(VERSION 3.8)
project(reach_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/reach_bringup)

ament_package()
