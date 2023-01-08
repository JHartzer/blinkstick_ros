cmake_minimum_required(VERSION 3.8)
project(blinkstick)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17 for blinkstickcpp
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(blinkstick_msgs REQUIRED)

# blinkstickcpp sources
set(blinkstickcpp_srcs
    include/blinkstickcpp/blinkstickcpp/src/blinkstick.cpp
    include/blinkstickcpp/blinkstickcpp/src/device.cpp
)

# Create library with EKF sources
add_library(blinkstickcpp ${blinkstickcpp_srcs})

# Generate Export Headd
include(GenerateExportHeader)
generate_export_header(blinkstickcpp 
    EXPORT_FILE_NAME ${PROJECT_BINARY_DIR}/blinkstickcpp/blinkstick/export.hpp
)

target_include_directories(blinkstickcpp PUBLIC
    include/blinkstickcpp/blinkstickcpp/include
    ${PROJECT_BINARY_DIR}/blinkstickcpp/
)

add_executable(${PROJECT_NAME} src/blinkstick.cpp)
ament_target_dependencies(${PROJECT_NAME} blinkstick_msgs)
target_link_libraries(${PROJECT_NAME} blinkstickcpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

ament_package()
