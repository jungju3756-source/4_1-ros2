cmake_minimum_required(VERSION 3.8)
project(lidarsave)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# lidarsave_node: /scan 구독(스캔영상 저장) + WASD 키보드(/vel_cmd 발행) 통합
add_executable(lidarsave_node src/lidarsave.cpp)
ament_target_dependencies(lidarsave_node rclcpp sensor_msgs geometry_msgs OpenCV)

install(TARGETS
  lidarsave_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
