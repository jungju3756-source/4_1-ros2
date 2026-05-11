#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <opencv2/opencv.hpp>
#include <termios.h>   // 터미널 설정 (linetracer_sim 참고)
#include <fcntl.h>
#include <thread>      // 키보드 입력 별도 스레드
#include <atomic>      // 스레드 간 안전한 공유 변수
#include <math.h>
#include <stdio.h>

// 이미지 설정 (2m x 2m: 500x500, 1m = 250px)
#define IMG_SIZE  500
#define CENTER    250

// 제어 파라미터
#define BASE_VEL  50
#define K_GAIN    0.3f

// ── 중심에서 픽셀 거리 ────────────────────────────────────────────
static float pixDist(int x, int y)
{
    float dx = x - CENTER, dy = y - CENTER;
    return sqrtf(dx*dx + dy*dy);
}

class LidarSimNode : public rclcpp::Node
{
public:
    LidarSimNode()
    : Node("lidarsim_node"),
      mode_(false),    // 초기: 정지 상태 (q=정지, s=시작)
      running_(true)
    {
        // /vel_cmd 발행 (Pi5 dxl_nano 구독)
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/vel_cmd", 10);

        // 입력 mp4 열기
        const std::string input_path = "/home/aim88/simulation/lidarplot2.mp4";
        cap_.open(input_path);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "영상을 열 수 없습니다: %s", input_path.c_str());
            rclcpp::shutdown();
            return;
        }

        // 처리 결과 mp4 저장
        writer_.open("lidarsim_output.mp4",
            cv::VideoWriter::fourcc('m','p','4','v'),
            10.0, cv::Size(IMG_SIZE, IMG_SIZE));

        // 10fps 타이머
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LidarSimNode::timerCb, this));

        // 키보드 스레드 시작 (linetracer_sim 방식과 동일)
        key_thread_ = std::thread(&LidarSimNode::keyboardLoop, this);

        RCLCPP_INFO(this->get_logger(),
            "LidarSim 시작. 's': 장애물회피 주행  'q': 정지");
        printf("=== LidarSim ===\n");
        printf("  s : 장애물 회피 주행 시작\n");
        printf("  q : 정지\n");
        printf("================\n");
    }

    ~LidarSimNode()
    {
        running_ = false;
        if (key_thread_.joinable()) key_thread_.join();

        // 정지 명령 발행
        auto msg = geometry_msgs::msg::Vector3();
        msg.x = 0.0; msg.y = 0.0;
        vel_pub_->publish(msg);

        if (cap_.isOpened())    cap_.release();
        if (writer_.isOpened()) writer_.release();
        cv::destroyAllWindows();
    }

private:
    // ── 키보드 스레드 (linetracer_sim 방식 그대로) ─────────────────
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
                    RCLCPP_INFO(this->get_logger(), "START - 장애물 회피 주행");
                }
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // 터미널 복원
    }

    // ── 메인 타이머 콜백 (10Hz) ────────────────────────────────────
    void timerCb()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "영상 종료. 정지 명령 발행.");
            auto msg = geometry_msgs::msg::Vector3();
            msg.x = 0.0; msg.y = 0.0;
            vel_pub_->publish(msg);
            rclcpp::shutdown();
            return;
        }

        // 입력 영상 크기 맞추기
        if (frame.cols != IMG_SIZE || frame.rows != IMG_SIZE) {
            cv::resize(frame, frame, cv::Size(IMG_SIZE, IMG_SIZE));
        }

        // ── 1. 장애물 회피 알고리즘 ──────────────────────────────
        int lx = -1, ly = -1;   // 좌측 최단거리 장애물
        int rx = -1, ry = -1;   // 우측 최단거리 장애물
        float min_dist_l = 1e9f;
        float min_dist_r = 1e9f;

        // 전방 180도 = 이미지 상반부 (y < CENTER)
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

        // 장애물 없는 쪽: 해당 방향 경계로 대체 (슬라이드 p5 참고)
        bool no_left  = (lx == -1);
        bool no_right = (rx == -1);
        if (no_left)  { lx =          0; ly = CENTER - 1; }
        if (no_right) { rx = IMG_SIZE-1; ry = CENTER - 1; }

        // ── 2. 에러 계산 ──────────────────────────────────────────
        int error = 0;
        int mid_x = CENTER;
        int mid_y = CENTER / 2;

        if (no_left && no_right) {
            error = 0;              // 전방 장애물 없음 → 직진
        } else {
            mid_x = (lx + rx) / 2;
            mid_y = (ly + ry) / 2;
            error = mid_x - CENTER; // >0: 우측 공간 넓음→우회전 / <0: 좌측 넓음→좌회전
        }

        // ── 3. 속도 명령 계산 ────────────────────────────────────
        float left_vel  =  (float)BASE_VEL + error * K_GAIN;
        float right_vel = -((float)BASE_VEL - error * K_GAIN);

        // mode_=true(s 키)일 때만 발행, false(q 키)면 정지
        auto msg = geometry_msgs::msg::Vector3();
        if (mode_) {
            msg.x = left_vel;
            msg.y = right_vel;
        } else {
            msg.x = 0.0;
            msg.y = 0.0;
        }
        vel_pub_->publish(msg);

        // ── 4. 시각화 ─────────────────────────────────────────────
        cv::Mat vis = frame.clone();

        // 전방/좌우 영역 분할선 (회색)
        cv::line(vis, cv::Point(0, CENTER), cv::Point(IMG_SIZE, CENTER),
                 cv::Scalar(180,180,180), 1);
        cv::line(vis, cv::Point(CENTER, 0), cv::Point(CENTER, IMG_SIZE),
                 cv::Scalar(180,180,180), 1);

        // 좌측 최단거리 장애물 화살표 (녹색)
        if (!no_left) {
            cv::arrowedLine(vis, cv::Point(CENTER, CENTER),
                            cv::Point(lx, ly), cv::Scalar(0,200,0), 2, 8, 0, 0.2);
            cv::circle(vis, cv::Point(lx, ly), 5, cv::Scalar(0,200,0), -1);
        }

        // 우측 최단거리 장애물 화살표 (빨간)
        if (!no_right) {
            cv::arrowedLine(vis, cv::Point(CENTER, CENTER),
                            cv::Point(rx, ry), cv::Scalar(0,0,200), 2, 8, 0, 0.2);
            cv::circle(vis, cv::Point(rx, ry), 5, cv::Scalar(0,0,200), -1);
        }

        // 중앙 방향 화살표 (파란 = 로봇 진행 방향)
        cv::arrowedLine(vis, cv::Point(CENTER, CENTER),
                        cv::Point(mid_x, mid_y), cv::Scalar(255,0,0), 3, 8, 0, 0.3);

        // 라이다 십자가
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

        cv::imshow("LidarSim (장애물 회피)", vis);
        cv::waitKey(1);

        if (writer_.isOpened()) writer_.write(vis);

        printf("[%s] error=%4d | L=%5.1f R=%5.1f\n",
               mode_.load() ? "RUN " : "STOP", error, left_vel, right_vel);
    }

    // 멤버 변수
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    cv::VideoWriter  writer_;
    std::atomic<bool> mode_;    // true=주행(s), false=정지(q) — 스레드 안전
    std::atomic<bool> running_; // 키보드 스레드 실행 플래그
    std::thread key_thread_;    // 키보드 입력 별도 스레드
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSimNode>());
    rclcpp::shutdown();
    return 0;
}
