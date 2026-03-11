cmake_minimum_required(VERSION 3.8)
project(linedetect_wsl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(linedetect_wsl src/sub.cpp src/main.cpp)
ament_target_dependencies(linedetect_wsl rclcpp std_msgs sensor_msgs cv_bridge OpenCV)

#Install
install(TARGETS
  linedetect_wsl
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()