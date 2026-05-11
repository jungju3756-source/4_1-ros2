#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <chrono>
#include <termios.h>  // 터미널 입력 설정 (raw mode)
#include <fcntl.h>    // 파일 디스크립터 제어
#include <thread>     // 키보드 입력 별도 스레드
#include <atomic>     // 스레드 간 안전한 공유 변수

class LineTrackerProcessor : public rclcpp::Node {
public:
    // mode_: 주행 여부 (s=true, q=false), k_: P제어 게인, base_vel_: 기본 속도
    LineTrackerProcessor() : Node("line_tracker_node"), mode_(false), k_(0.10), base_vel_(800), running_(true) {
        // ROS 파라미터 선언 (런타임에 ros2 param set으로 변경 가능)
        this->declare_parameter("k", 0.17);
        this->declare_parameter("base_vel", 100);

        k_ = this->get_parameter("k").as_double();
        base_vel_ = this->get_parameter("base_vel").as_int();

        // 구독: BestEffort (영상 손실 허용, 지연 최소화)
        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        // 발행: Reliable (모터 명령은 손실 없이 전달)
        auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));  // Reliable (dxl_wsl과 동일)

        // Image_Topic 구독 → image_callback 호출
        raw_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "Image_Topic", sub_qos,
            std::bind(&LineTrackerProcessor::image_callback, this, std::placeholders::_1));

        // 모터 속도 명령 퍼블리셔 (x=좌측, y=우측 RPM)
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", pub_qos);

        // 초기 추적 위치 설정 (화면 중앙 x=320, ROI 중간 y=45)
        last_line_x_ = 320.0;
        last_line_y_ = 45.0;

        RCLCPP_INFO(this->get_logger(), "분석 및 제어 노드 시작. 's': 주행, 'q': 정지");

        // 키보드 입력을 메인 스핀과 별도 스레드로 처리
        key_thread_ = std::thread(&LineTrackerProcessor::keyboardLoop, this);
    }

    ~LineTrackerProcessor() {
        running_ = false;                          // 키보드 스레드 종료 신호
        if (key_thread_.joinable()) key_thread_.join();
    }

private:
    // --- 1. 전처리 함수 (ROI 설정 및 이진화) ---
    cv::Mat setROI(cv::Mat &frame) {
        // 하단 1/4 영역 추출 (라인이 있는 영역만 처리해 연산량 감소)
        cv::Rect roi_rect(0, frame.rows * 3 / 4, frame.cols, frame.rows / 4);
        cv::Mat roi = frame(roi_rect).clone();

        cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
        // ROI 내부 평균 밝기를 100으로 정규화 (조명 변화 대응)
        roi += cv::Scalar(100) - cv::mean(roi);
        // 임계값 150으로 이진화 (흰 라인=255, 배경=0)
        cv::threshold(roi, roi, 150, 255, cv::THRESH_BINARY);

        return roi;
    }

    // --- 2. 라인 탐색 함수 (가장 가까운 객체 index 반환) ---
    int findLine(cv::Mat &bin_roi, cv::Mat &stats, cv::Mat &centroids) {
        // Connected Component로 흰색 blob 분리
        int n_labels = cv::connectedComponentsWithStats(bin_roi, labels_, stats, centroids);

        // [1단계] 현재 중심점과 가장 가까운 후보 찾기
        int min_index = -1;
        double min_dist = static_cast<double>(bin_roi.cols);

        for (int i = 1; i < n_labels; i++) {  // 0번은 배경이므로 1부터
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area > 100) { // 면적 100 미만은 노이즈로 제거
                double cx = centroids.at<double>(i, 0);
                double cy = centroids.at<double>(i, 1);
                double dist = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_line_x_, last_line_y_));

                // 이전 라인 위치에서 150px 이내 후보만 유효
                if (dist < min_dist && dist <= 150.0) {
                    min_dist = dist;
                    min_index = i;
                }
            }
        }

        // 150px 이내에 후보가 있으면 중심점 1차 갱신
        if (min_index != -1 && min_dist <= 150.0) {
            last_line_x_ = centroids.at<double>(min_index, 0);
            last_line_y_ = centroids.at<double>(min_index, 1);
        }

        // [2단계] 갱신된 중심점에서 다시 한번 가장 가까운 blob 확정
        int idx = -1;
        double best = static_cast<double>(bin_roi.cols);

        for (int i = 1; i < stats.rows; i++) {
            double cx = centroids.at<double>(i, 0);
            double cy = centroids.at<double>(i, 1);
            double d = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_line_x_, last_line_y_));

            if (d < best) {
                best = d;
                idx = i;
            }
        }

        // 최종 거리가 30px 초과이면 라인 미검출로 처리 (-1 반환)
        if (best > 30.0) {
            idx = -1;
        }
        return idx;
    }

    // --- 3. 시각화 함수 ---
    cv::Mat drawResult(cv::Mat &bin_roi, cv::Mat &stats, int best_idx) {
        cv::Mat display;
        cv::cvtColor(bin_roi, display, cv::COLOR_GRAY2BGR);  // 이진 이미지를 컬러로 변환

        for (int i = 1; i < stats.rows; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < 100) continue;  // 노이즈 제거

            int l = stats.at<int>(i, 0), t = stats.at<int>(i, 1), w = stats.at<int>(i, 2), h = stats.at<int>(i, 3);

            if (i == best_idx) {
                // 검출된 라인 (빨간색)
                cv::rectangle(display, cv::Rect(l, t, w, h), cv::Scalar(0, 0, 255), 2);
            } else {
                // 후보 노이즈 (파란색)
                cv::rectangle(display, cv::Rect(l, t, w, h), cv::Scalar(255, 0, 0), 1);
            }
        }
        // 최종 추적 지점 점 찍기
        cv::circle(display, cv::Point(static_cast<int>(last_line_x_), static_cast<int>(last_line_y_)), 3, cv::Scalar(0, 0, 255), -1);

        return display;
    }

    // --- 4. 메인 콜백 함수 ---
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        auto startTime = std::chrono::steady_clock::now();  // 처리 시간 측정 시작

        // 매 콜백마다 파라미터 갱신 (런타임 튜닝 반영)
        k_ = this->get_parameter("k").as_double();
        base_vel_ = this->get_parameter("base_vel").as_int();

        // 압축 영상 복원
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        // 전처리: ROI 추출 + 밝기 정규화 + 이진화
        cv::Mat bin_roi = setROI(frame);
        cv::Mat stats, centroids;
        // 라인 검출: 가장 가까운 blob의 index 반환 (-1이면 미검출)
        int best_idx = findLine(bin_roi, stats, centroids);
        // 시각화 이미지 생성
        cv::Mat display = drawResult(bin_roi, stats, best_idx);

        // P 제어: error = 화면 중심 - 라인 x좌표
        // error > 0: 라인이 왼쪽 → 우회전 / error < 0: 라인이 오른쪽 → 좌회전
        double error = (bin_roi.cols / 2.0) - last_line_x_;
        geometry_msgs::msg::Vector3 vel_msg;
        if (mode_) {
            // 좌측 모터: 기본속도 - 오차 보정 / 우측 모터: 기본속도 + 오차 보정 (부호 반전)
            vel_msg.x = base_vel_ - error * k_;
            vel_msg.y = -(base_vel_ + error * k_);
        } else {
            // 정지 모드: 속도 0 발행
            vel_msg.x = 0; vel_msg.y = 0;
        }
        vel_pub_->publish(vel_msg);  // 모터 명령 발행

        // 처리 시간 측정 종료 (publish까지 포함, imshow 제외)
        auto endTime = std::chrono::steady_clock::now();
        float totalTime = std::chrono::duration<float, std::milli>(endTime - startTime).count();
        RCLCPP_INFO(this->get_logger(), "err:%.2lf lvel:%.2f rvel:%.2f time:%.2f", error,vel_msg.x,vel_msg.y, totalTime);

        cv::imshow("1. Raw Video", frame);
        cv::imshow("2. Binary Debug", display);
        cv::waitKey(1);
    }


    // 별도 스레드에서 키보드 입력 대기
    void keyboardLoop() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);   // 현재 터미널 설정 저장
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO); // raw mode: 엔터 없이 즉시 입력 처리, 에코 끄기
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (running_) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);
            struct timeval tv = {0, 100000}; // 100ms timeout (블로킹 방지)
            if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
                char ch = getchar();
                if (ch == 'q') { mode_ = false; RCLCPP_WARN(this->get_logger(), "STOP"); }
                else if (ch == 's') { mode_ = true; RCLCPP_INFO(this->get_logger(), "START"); }
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // 터미널 설정 복원
    }


    // 멤버 변수
    double last_line_x_, last_line_y_;  // 이전 프레임의 라인 중심 좌표 (추적 연속성 유지)
    std::atomic<bool> mode_;            // 주행 모드 (true=주행, false=정지), 스레드 안전
    double k_;                          // P 제어 게인
    int base_vel_;                      // 기본 전진 속도 (RPM)
    cv::Mat labels_;                    // connectedComponents 레이블 맵 (재사용)
    std::atomic<bool> running_;         // 키보드 스레드 실행 플래그
    std::thread key_thread_;            // 키보드 입력 스레드
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr raw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vel_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineTrackerProcessor>());
    rclcpp::shutdown();
    return 0;
}
