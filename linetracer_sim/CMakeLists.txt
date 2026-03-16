cmake_minimum_required(VERSION 3.8)
project(linetracer_sim)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(video_publisher src/video_publisher.cpp)
ament_target_dependencies(video_publisher rclcpp sensor_msgs std_msgs cv_bridge OpenCV)

add_executable(line_controller src/line_controller.cpp)
ament_target_dependencies(line_controller rclcpp sensor_msgs std_msgs geometry_msgs cv_bridge OpenCV)

add_executable(dxl_subscriber src/dxl_subscriber.cpp)
ament_target_dependencies(dxl_subscriber rclcpp geometry_msgs)

install(TARGETS
  video_publisher
  line_controller
  dxl_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
