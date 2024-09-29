cmake_minimum_required(VERSION 3.8)
project(
    libreach
    VERSION 0.1.2
    LANGUAGES CXX
    DESCRIPTION "C++ library used to communicate with Reach Robotics devices"
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(Boost REQUIRED COMPONENTS system)
find_package(ament_cmake REQUIRED)

add_library(libreach SHARED)
target_sources(
    libreach
    PRIVATE
        src/client.cpp
        src/cobs.cpp
        src/crc.cpp
        src/driver.cpp
        src/packet.cpp
        src/serial_client.cpp
        src/serial_driver.cpp
        src/udp_client.cpp
        src/udp_driver.cpp
    PUBLIC
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
        FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/include/libreach/client.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/include/libreach/device_id.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/include/libreach/driver.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/include/libreach/mode.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/include/libreach/packet_id.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/include/libreach/packet.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/include/libreach/serial_client.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/include/libreach/serial_driver.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/include/libreach/udp_client.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/include/libreach/udp_driver.hpp
)

target_compile_features(libreach PUBLIC cxx_std_20)
target_link_libraries(libreach PUBLIC Boost::boost PRIVATE Boost::system)
set_target_properties(libreach PROPERTIES PREFIX "")

set(EXAMPLES
    alpha5_example
    bravo7_example
    multiple_workers
    request_packets
    send_packets
)

foreach(example IN ITEMS ${EXAMPLES})
    add_executable(${example} examples/${example}.cpp)
    add_dependencies(${example} libreach)
    target_link_libraries(${example} PUBLIC libreach)
    set_target_properties(
        ${example}
        PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/examples
    )
endforeach()

install(
    TARGETS libreach
    EXPORT libreachTargets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FILE_SET HEADERS
)

ament_export_targets(libreachTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Boost)

ament_package()
