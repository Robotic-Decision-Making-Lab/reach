cmake_minimum_required(VERSION 3.8)
project(reach_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    rclcpp
    libreach
    rclcpp_lifecycle
    pluginlib
    hardware_interface
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach()

add_library(reach_hardware SHARED)
target_sources(reach_hardware PRIVATE src/alpha_5_hardware.cpp)

target_include_directories(
    reach_hardware
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(reach_hardware PUBLIC cxx_std_17)
ament_target_dependencies(reach_hardware ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface alpha_5_hardware.xml)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)

    ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(
    TARGETS reach_hardware
    EXPORT export_reach_hardware
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

ament_export_targets(export_reach_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
