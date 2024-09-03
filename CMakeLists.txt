cmake_minimum_required(VERSION 3.8)
project(ros2_serialsample1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(sample1_sources
src/serialsample1.cpp
src/serial.cpp
include/commonsft.hpp
include/serial.hpp)

add_executable(serialsample1 ${sample1_sources})

target_include_directories(serialsample1 PRIVATE ./include)

ament_target_dependencies(serialsample1 rclcpp) 

install(TARGETS
    serialsample1
    DESTINATION lib/${PROJECT_NAME})

ament_package()
