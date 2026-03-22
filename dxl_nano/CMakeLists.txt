cmake_minimum_required(VERSION 3.5)
project(dxl_nano)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)
include_directories(/usr/local/include/dynamixel_sdk)

# Build
add_executable(sub src/sub.cpp src/dxl.cpp)
ament_target_dependencies(sub rclcpp geometry_msgs)
target_link_libraries(sub dxl_x64_cpp)
#target_include_directories(sub PUBLIC /usr/local/include/dynamixel_sdk)

install(TARGETS
  sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()

