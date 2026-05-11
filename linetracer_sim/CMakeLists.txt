cmake_minimum_required(VERSION 3.8)
project(linetracer_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(linetracer linetracer.cpp)
ament_target_dependencies(linetracer rclcpp sensor_msgs geometry_msgs cv_bridge OpenCV)

install(TARGETS linetracer
  DESTINATION lib/${PROJECT_NAME})

ament_package()
