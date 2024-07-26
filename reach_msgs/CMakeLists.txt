cmake_minimum_required(VERSION 3.8)
project(reach_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  msg/MultiDOFImpedanceCommand.msg
  msg/MultiDOFImpedanceStateStamped.msg
)

rosidl_generate_interfaces(reach_msgs
  ${msg_files}
  DEPENDENCIES
    builtin_interfaces
    std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
