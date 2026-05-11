cmake_minimum_required(VERSION 3.8)
project(lidarsim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# lidarsim_node: mp4 재생 → 장애물 회피 알고리즘 → /vel_cmd 발행 + 결과 mp4 저장
add_executable(lidarsim_node src/lidarsim.cpp)
ament_target_dependencies(lidarsim_node rclcpp geometry_msgs OpenCV)

install(TARGETS
  lidarsim_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
