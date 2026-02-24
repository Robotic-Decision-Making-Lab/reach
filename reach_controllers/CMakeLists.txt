cmake_minimum_required(VERSION 3.23)
project(reach_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(reach_msgs REQUIRED)
find_package(ament_cmake REQUIRED)

generate_parameter_library(impedance_controller_parameters
  src/impedance_controller_parameters.yaml
)

add_library(reach_controllers SHARED)
target_sources(
    reach_controllers
    PRIVATE src/impedance_controller.cpp
    PUBLIC
        FILE_SET HEADERS
            BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            FILES
                ${CMAKE_CURRENT_SOURCE_DIR}/include/reach_controllers/impedance_controller.hpp
)
target_compile_features(reach_controllers PUBLIC cxx_std_20)
target_link_libraries(
    reach_controllers
    PUBLIC
        impedance_controller_parameters
        controller_interface::controller_interface
        hardware_interface::hardware_interface
        hardware_interface::mock_components
        pluginlib::pluginlib
        realtime_tools::realtime_tools
        realtime_tools::thread_priority
        ${reach_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface reach_controllers.xml)

install(
    TARGETS reach_controllers impedance_controller_parameters
    EXPORT export_reach_controllers
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

ament_export_targets(export_reach_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(
  "controller_interface"
  "hardware_interface"
  "pluginlib"
  "generate_parameter_library"
  "realtime_tools"
  "reach_msgs"
)

ament_package()
