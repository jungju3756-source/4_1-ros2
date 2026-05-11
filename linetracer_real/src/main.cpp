#include "rclcpp/rclcpp.hpp"                          // ROS 2 C++ 클라이언트 라이브러리 헤더 (Node, spin, init 등 제공)
#include "linetracer_real/linetracer_real.hpp"        // LineTrackerProcessor 클래스 선언 헤더

int main(int argc, char** argv) {                     // 프로그램 진입점 (argc: 인자 개수, argv: 인자 배열)
    rclcpp::init(argc, argv);                         // ROS 2 런타임 초기화 (통신 미들웨어 DDS 초기화 및 인자 파싱)
    rclcpp::spin(std::make_shared<LineTrackerProcessor>()); // 노드 인스턴스를 shared_ptr로 생성 후 spin → 콜백 루프 진입(Ctrl+C까지 블로킹)
    rclcpp::shutdown();                               // ROS 2 런타임 종료 (DDS 정리, 리소스 해제)
    return 0;                                         // 정상 종료 반환값
}
