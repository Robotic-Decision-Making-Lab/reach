cmake_minimum_required(VERSION 3.24)
project(reach_ip_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_search_module(gstreamer REQUIRED IMPORTED_TARGET gstreamer-1.0)
pkg_search_module(gstreamer-app REQUIRED IMPORTED_TARGET gstreamer-app-1.0)

generate_parameter_library(gstreamer_proxy_parameters
    src/gstreamer_proxy_parameters.yaml
)

add_library(gstreamer_proxy_library SHARED)

target_sources(
    gstreamer_proxy_library
    PRIVATE src/gstreamer_proxy.cpp
    PUBLIC
        FILE_SET HEADERS
            BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            FILES
                ${CMAKE_CURRENT_SOURCE_DIR}/include/reach_ip_camera/gstreamer_proxy.hpp
)
target_link_libraries(
    gstreamer_proxy_library
    PRIVATE gstreamer_proxy_parameters
    PUBLIC PkgConfig::gstreamer
    PUBLIC PkgConfig::gstreamer-app
)
target_compile_features(gstreamer_proxy_library PUBLIC cxx_std_20)
target_link_libraries(
    gstreamer_proxy_library
    PUBLIC
        camera_info_manager::camera_info_manager
        cv_bridge::cv_bridge
        image_transport::image_transport
        rclcpp::rclcpp
        rclcpp_components::component
        rclcpp_components::component_manager
        ${geometry_msgs_TARGETS}
)

rclcpp_components_register_node(gstreamer_proxy_library
    PLUGIN "reach::GStreamerProxy"
    EXECUTABLE gstreamer_proxy
)

install(
    TARGETS gstreamer_proxy_library gstreamer_proxy_parameters
    EXPORT export_gstreamer_proxy_library
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

ament_export_targets(export_gstreamer_proxy_library HAS_LIBRARY_TARGET)
ament_export_dependencies(
  "rclcpp"
  "geometry_msgs"
  "rclcpp_components"
  "cv_bridge"
  "camera_info_manager"
  "image_transport"
)

ament_package()
