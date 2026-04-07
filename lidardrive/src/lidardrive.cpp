#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <opencv2/opencv.hpp>
#include <termios.h>   // linetracer_sim 방식 키보드
#include <fcntl.h>
#include <thread>
#include <atomic>
#include <math.h>
#include <stdio.h>

// 스캔 영상 설정 (2m x 2m: 500x500px, 1m = 250px)
#define IMG_SIZE   500
#define CENTER     250
#define SCALE      250.0f   // px/m
#define MAX_RANGE  1.0f     // 반경 1m 이내만 표시/사용

// 제어 파라미터
#define BASE_VEL   50
#define K_GAIN     0.15f    // error(픽셀) → 속도 변환 게인

// ── 중심에서 픽셀 거리 ────────────────────────────────────────────
static float pixDist(int x, int y)
{
    float dx = x - CENTER, dy = y - CENTER;
    return sqrtf(dx*dx + dy*dy);
}

class LidarDriveNode : public rclcpp::Node
{
public:
    LidarDriveNode()
    : Node("lidardrive_node"),
      mode_(false),
      running_(true)
    {
        // /scan 구독 (Pi5 sllidar_node 발행)
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LidarDriveNode::scanCb, this, std::placeholders::_1));

        // /vel_cmd 발행 (Pi5 dxl_nano 구독)
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/vel_cmd", 10);

        // 결과 mp4 저장 (10fps)
        writer_.open("lidardrive_output.mp4",
            cv::VideoWriter::fourcc('m','p','4','v'),
            10.0, cv::Size(IMG_SIZE, IMG_SIZE));

        // 키보드 스레드 시작 (linetracer_sim 방식)
        key_thread_ = std::thread(&LidarDriveNode::keyboardLoop, this);

        RCLCPP_INFO(this->get_logger(),
            "LidarDrive 시작. /scan 구독 → 장애물회피 → /vel_cmd 발행");
        printf("=== LidarDrive ===\n");
        printf("  s : 장애물 회피 자율주행 시작\n");
        printf("  q : 정지\n");
        printf("==================\n");
    }

    ~LidarDriveNode()
    {
        running_ = false;
        if (key_thread_.joinable()) key_thread_.join();

        // 정지 명령
        auto msg = geometry_msgs::msg::Vector3();
        msg.x = 0.0; msg.y = 0.0;
        vel_pub_->publish(msg);

        if (writer_.isOpened()) writer_.release();
        cv::destroyAllWindows();
    }

private:
    // ── 키보드 스레드 (linetracer_sim과 동일 방식) ─────────────────
    void keyboardLoop()
    {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);  // raw mode: 엔터 없이 즉시 입력
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (running_) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);
            struct timeval tv = {0, 100000};  // 100ms timeout (블로킹 방지)
            if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
                char ch = getchar();
                if (ch == 'q' || ch == 'Q') {
                    mode_ = false;
                    RCLCPP_WARN(this->get_logger(), "STOP");
                } else if (ch == 's' || ch == 'S') {
                    mode_ = true;
                    RCLCPP_INFO(this->get_logger(), "START - 장애물 회피 자율주행");
                }
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    // ── /scan 콜백: 영상 생성 + 장애물 회피 + vel_cmd 발행 ─────────
    void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        int count = static_cast<int>(scan->ranges.size());

        // ── 1. 스캔 영상 생성 (2m x 2m, 흰 배경) ─────────────────
        cv::Mat frame(IMG_SIZE, IMG_SIZE, CV_8UC3, cv::Scalar(255, 255, 255));

        // 경계 사각형 (파란)
        cv::rectangle(frame, cv::Point(1,1), cv::Point(IMG_SIZE-2, IMG_SIZE-2),
                      cv::Scalar(255,0,0), 2);

        // 장애물 점 그리기 + 장애물 회피를 위한 픽셀 좌표 수집
        for (int i = 0; i < count; i++) {
            float range = scan->ranges[i];
            if (std::isinf(range) || std::isnan(range) || range <= 0.0f) continue;
            if (range > MAX_RANGE) continue;   // 반경 1m 초과 제외

            float angle_rad = scan->angle_min + scan->angle_increment * i;
            float r_pixel   = range * SCALE;

            // 슬라이드 수식: px = CENTER + R*sin(PI-θ), py = CENTER - R*cos(PI-θ)
            int px = static_cast<int>(CENTER + r_pixel * sinf(M_PI - angle_rad));
            int py = static_cast<int>(CENTER - r_pixel * cosf(M_PI - angle_rad));

            if (px >= 0 && px < IMG_SIZE && py >= 0 && py < IMG_SIZE) {
                cv::circle(frame, cv::Point(px, py), 2, cv::Scalar(0,0,255), -1);
            }
        }

        // ── 2. 장애물 회피 알고리즘 ──────────────────────────────
        // 전방 180도 = 이미지 상반부 (y < CENTER) 에서 빨간 점 탐색
        int lx = -1, ly = -1;
        int rx = -1, ry = -1;
        float min_dist_l = 1e9f;
        float min_dist_r = 1e9f;

        for (int y = 0; y < CENTER; y++) {
            for (int x = 0; x < IMG_SIZE; x++) {
                cv::Vec3b px = frame.at<cv::Vec3b>(y, x);
                // 빨간 점 감지 (BGR): R>150, G<80, B<80
                if (px[2] > 150 && px[1] < 80 && px[0] < 80) {
                    float d = pixDist(x, y);
                    if (x < CENTER) {           // 좌측 ½
                        if (d < min_dist_l) { min_dist_l = d; lx = x; ly = y; }
                    } else {                    // 우측 ½
                        if (d < min_dist_r) { min_dist_r = d; rx = x; ry = y; }
                    }
                }
            }
        }

        // 장애물 없는 쪽: 경계값으로 대체 (슬라이드 p5 참고)
        bool no_left  = (lx == -1);
        bool no_right = (rx == -1);
        if (no_left)  { lx =          0; ly = CENTER - 1; }
        if (no_right) { rx = IMG_SIZE-1; ry = CENTER - 1; }

        // ── 3. 에러 계산 ──────────────────────────────────────────
        int error = 0;
        int mid_x = CENTER;
        int mid_y = CENTER / 2;

        if (no_left && no_right) {
            error = 0;  // 전방 장애물 없음 → 직진
        } else {
            mid_x = (lx + rx) / 2;
            mid_y = (ly + ry) / 2;
            // error > 0: 우측 공간 넓음 → 우회전
            // error < 0: 좌측 공간 넓음 → 좌회전
            error = mid_x - CENTER;
        }

        // ── 4. 속도 명령 계산 및 발행 ────────────────────────────
        float left_vel  =  (float)BASE_VEL + error * K_GAIN;
        float right_vel = -((float)BASE_VEL - error * K_GAIN);

        auto msg = geometry_msgs::msg::Vector3();
        if (mode_) {
            msg.x = left_vel;
            msg.y = right_vel;
        } else {
            msg.x = 0.0;
            msg.y = 0.0;
        }
        vel_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
            "[%s] err=%d L=%.1f R=%.1f",
            mode_.load() ? "RUN" : "STOP", error, msg.x, msg.y);

        // ── 5. 시각화 ─────────────────────────────────────────────
        cv::Mat vis = frame.clone();

        // 전방/좌우 영역 분할선 (회색)
        cv::line(vis, cv::Point(0, CENTER), cv::Point(IMG_SIZE, CENTER),
                 cv::Scalar(150,150,150), 1);
        cv::line(vis, cv::Point(CENTER, 0), cv::Point(CENTER, CENTER),
                 cv::Scalar(150,150,150), 1);

        // 좌측 최단 장애물 화살표 (녹색)
        if (!no_left) {
            cv::arrowedLine(vis, cv::Point(CENTER, CENTER),
                            cv::Point(lx, ly), cv::Scalar(0,200,0), 2, 8, 0, 0.2);
            cv::circle(vis, cv::Point(lx, ly), 5, cv::Scalar(0,200,0), -1);
        }

        // 우측 최단 장애물 화살표 (빨간)
        if (!no_right) {
            cv::arrowedLine(vis, cv::Point(CENTER, CENTER),
                            cv::Point(rx, ry), cv::Scalar(0,0,200), 2, 8, 0, 0.2);
            cv::circle(vis, cv::Point(rx, ry), 5, cv::Scalar(0,0,200), -1);
        }

        // 중앙 방향 화살표 (파란 = 로봇 진행 방향)
        cv::arrowedLine(vis, cv::Point(CENTER, CENTER),
                        cv::Point(mid_x, mid_y), cv::Scalar(255,0,0), 3, 8, 0, 0.3);

        // 라이다 위치 십자가 (검정)
        cv::line(vis, cv::Point(CENTER, CENTER-10), cv::Point(CENTER, CENTER+10),
                 cv::Scalar(0,0,0), 2);
        cv::line(vis, cv::Point(CENTER-10, CENTER), cv::Point(CENTER+10, CENTER),
                 cv::Scalar(0,0,0), 2);

        // 상태 텍스트
        char buf[128];
        snprintf(buf, sizeof(buf), "[%s] err=%d  L=%.1f R=%.1f",
                 mode_ ? "RUN" : "STOP", error, left_vel, right_vel);
        cv::putText(vis, buf, cv::Point(5, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    mode_ ? cv::Scalar(0,128,0) : cv::Scalar(0,0,200), 1);

        cv::imshow("LidarDrive (2m x 2m)", vis);
        cv::waitKey(1);

        if (writer_.isOpened()) writer_.write(vis);
    }

    // 멤버 변수
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr    vel_pub_;
    cv::VideoWriter  writer_;
    std::atomic<bool> mode_;     // true=자율주행(s), false=정지(q)
    std::atomic<bool> running_;  // 키보드 스레드 플래그
    std::thread key_thread_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarDriveNode>());
    rclcpp::shutdown();
    return 0;
}
