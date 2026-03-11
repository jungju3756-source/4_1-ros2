cmake_minimum_required(VERSION 3.5)
project(linedetect_nano)

# C++17 사용
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

# 실행파일 (pub)
add_executable(pub src/pub.cpp)

# 의존성 연결
ament_target_dependencies(pub
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
)

# 설치 경로
install(TARGETS pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
