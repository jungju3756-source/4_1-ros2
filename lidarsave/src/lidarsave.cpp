#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <opencv2/opencv.hpp>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>

// 2m x 2m 영역: 500x500 픽셀, 1m = 250px
#define SCALE     250.0f
#define MAX_RANGE 1.0f
#define IMG_SIZE  500
#define CENTER    250
#define BASE_VEL  50

class LidarSaveNode : public rclcpp::Node
{
public:
    LidarSaveNode() : Node("lidarsave_node")
    {
        // /scan 구독
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LidarSaveNode::scanCb, this, std::placeholders::_1));

        // /vel_cmd 발행 (라즈베리파이 dxl_nano가 구독)
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/vel_cmd", 10);

        // 키보드 입력 타이머 (100ms 주기)
        key_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LidarSaveNode::keyboardCb, this));

        // mp4 저장 (10fps)
        video_writer_.open("lidar_save_output.mp4",
            cv::VideoWriter::fourcc('m','p','4','v'),
            10.0, cv::Size(IMG_SIZE, IMG_SIZE));

        // 터미널 raw 모드
        tcgetattr(STDIN_FILENO, &orig_termios_);
        struct termios raw = orig_termios_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        printf("=== LidarSave + 키보드 제어 시작 ===\n");
        printf("  w : 전진  |  s : 후진\n");
        printf("  a : 좌회전|  d : 우회전\n");
        printf("  space : 정지  |  q : 종료\n");
        printf("=====================================\n");
    }

    ~LidarSaveNode()
    {
        // 정지 명령 발행 후 종료
        auto msg = geometry_msgs::msg::Vector3();
        msg.x = 0.0; msg.y = 0.0;
        vel_pub_->publish(msg);

        if (video_writer_.isOpened()) video_writer_.release();
        cv::destroyAllWindows();
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    }

private:
    // ── 키보드 콜백 ───────────────────────────────────────────────
    void keyboardCb()
    {
        char key = 0;
        int n = read(STDIN_FILENO, &key, 1);
        if (n <= 0) return;

        auto msg = geometry_msgs::msg::Vector3();

        switch (key) {
            case 'w': case 'W':
                msg.x =  BASE_VEL; msg.y = -BASE_VEL;
                printf("[전진]   L=%d R=%d\n", BASE_VEL, -BASE_VEL);
                break;
            case 's': case 'S':
                msg.x = -BASE_VEL; msg.y =  BASE_VEL;
                printf("[후진]   L=%d R=%d\n", -BASE_VEL, BASE_VEL);
                break;
            case 'a': case 'A':
                msg.x = -BASE_VEL; msg.y = -BASE_VEL;
                printf("[좌회전] L=%d R=%d\n", -BASE_VEL, -BASE_VEL);
                break;
            case 'd': case 'D':
                msg.x =  BASE_VEL; msg.y =  BASE_VEL;
                printf("[우회전] L=%d R=%d\n", BASE_VEL, BASE_VEL);
                break;
            case ' ':
                msg.x = 0.0; msg.y = 0.0;
                printf("[정지]\n");
                break;
            case 'q': case 'Q':
                msg.x = 0.0; msg.y = 0.0;
                vel_pub_->publish(msg);
                printf("종료합니다.\n");
                rclcpp::shutdown();
                return;
            default:
                return;  // 그 외 키는 무시
        }

        vel_pub_->publish(msg);
    }

    // ── 라이다 스캔 콜백 ─────────────────────────────────────────
    void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        int count = static_cast<int>(scan->ranges.size());

        // 흰색 배경
        cv::Mat frame(IMG_SIZE, IMG_SIZE, CV_8UC3, cv::Scalar(255, 255, 255));

        // 2m×2m 경계 (파란색 테두리)
        cv::rectangle(frame, cv::Point(1, 1), cv::Point(IMG_SIZE-2, IMG_SIZE-2),
                      cv::Scalar(255, 0, 0), 2);

        // 라이다 위치 십자가 (검정)
        cv::line(frame, cv::Point(CENTER, CENTER-12), cv::Point(CENTER, CENTER+12),
                 cv::Scalar(0, 0, 0), 2);
        cv::line(frame, cv::Point(CENTER-12, CENTER), cv::Point(CENTER+12, CENTER),
                 cv::Scalar(0, 0, 0), 2);

        for (int i = 0; i < count; i++) {
            float range = scan->ranges[i];

            if (std::isinf(range) || std::isnan(range) || range <= 0.0f) continue;
            if (range > MAX_RANGE) continue;  // 반경 1m 초과 제외

            float angle_rad = scan->angle_min + scan->angle_increment * i;
            float r_pixel   = range * SCALE;

            // 슬라이드 수식: px = CENTER + R*sin(PI-θ), py = CENTER - R*cos(PI-θ)
            int px = static_cast<int>(CENTER + r_pixel * sinf(M_PI - angle_rad));
            int py = static_cast<int>(CENTER - r_pixel * cosf(M_PI - angle_rad));

            if (px >= 0 && px < IMG_SIZE && py >= 0 && py < IMG_SIZE) {
                cv::circle(frame, cv::Point(px, py), 2, cv::Scalar(0, 0, 255), -1);
            }
        }

        cv::imshow("LidarSave (2m x 2m)", frame);
        cv::waitKey(1);

        if (video_writer_.isOpened()) video_writer_.write(frame);
    }

    // 멤버 변수
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr    vel_pub_;
    rclcpp::TimerBase::SharedPtr key_timer_;
    cv::VideoWriter video_writer_;
    struct termios orig_termios_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSaveNode>());
    rclcpp::shutdown();
    return 0;
}
