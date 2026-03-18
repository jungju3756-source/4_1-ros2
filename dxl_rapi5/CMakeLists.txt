cmake_minimum_required(VERSION 3.8)
project(dxl_rapi5)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_executable(dxl_sub src/dxl_sub.cpp)
ament_target_dependencies(dxl_sub rclcpp geometry_msgs dynamixel_sdk)

install(TARGETS
  dxl_sub
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
