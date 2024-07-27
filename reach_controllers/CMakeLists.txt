cmake_minimum_required(VERSION 3.8)
project(reach_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    hardware_interface
    pluginlib
    generate_parameter_library
    realtime_tools
    reach_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(impedance_controller_parameters
  src/impedance_controller_parameters.yaml
)

add_library(reach_controllers SHARED)
target_sources(reach_controllers PRIVATE src/impedance_controller.cpp)

target_include_directories(
  reach_controllers
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE ${PROJECT_SOURCE_DIR}/src
)
target_compile_features(reach_controllers PUBLIC cxx_std_20)
target_link_libraries(
    reach_controllers
    PUBLIC impedance_controller_parameters
)
ament_target_dependencies(reach_controllers
  PUBLIC
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(controller_interface reach_controllers.xml)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
    TARGETS reach_controllers impedance_controller_parameters
    EXPORT export_reach_controllers
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

ament_export_targets(export_reach_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
