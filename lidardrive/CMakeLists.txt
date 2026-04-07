cmake_minimum_required(VERSION 3.8)
project(lidardrive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# lidardrive_node: /scan 구독 → 장애물회피 → /vel_cmd 발행 + 스캔영상 mp4 저장
add_executable(lidardrive_node src/lidardrive.cpp)
ament_target_dependencies(lidardrive_node rclcpp sensor_msgs geometry_msgs OpenCV)

install(TARGETS
  lidardrive_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
