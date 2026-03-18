cmake_minimum_required(VERSION 3.8)
project(linetracer_real)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(linetracer_real src/linetracer_real.cpp)
ament_target_dependencies(linetracer_real rclcpp sensor_msgs geometry_msgs OpenCV)

install(TARGETS
  linetracer_real
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
