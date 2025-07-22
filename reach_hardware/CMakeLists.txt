cmake_minimum_required(VERSION 3.23)
project(reach_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(libreach REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ament_cmake REQUIRED)

add_library(reach_hardware SHARED)
target_sources(
    reach_hardware
    PRIVATE src/alpha_5_hardware.cpp
    PUBLIC
        FILE_SET HEADERS
            BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            FILES
                ${CMAKE_CURRENT_SOURCE_DIR}/include/reach_hardware/alpha_5_hardware.hpp
)
target_compile_features(reach_hardware PUBLIC cxx_std_20)
target_link_libraries(
    reach_hardware
    PUBLIC
        hardware_interface::hardware_interface
        hardware_interface::mock_components
        libreach::libreach
        pluginlib::pluginlib
        rclcpp::rclcpp
        rclcpp_lifecycle::rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface alpha_5_hardware.xml)

install(
    TARGETS reach_hardware
    EXPORT export_reach_hardware
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    FILE_SET HEADERS
)

ament_export_targets(export_reach_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  "rclcpp"
  "rclcpp_lifecycle"
  "libreach"
  "pluginlib"
  "hardware_interface"
)

ament_package()
