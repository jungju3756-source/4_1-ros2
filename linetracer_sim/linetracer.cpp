#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <cmath>
#include <chrono>
#include <termios.h>
#include <fcntl.h>
#include <thread>
#include <atomic>

class LineTrackerProcessor : public rclcpp::Node {
public:
    LineTrackerProcessor() : Node("line_tracker_node"), mode_(false), k_(0.15), base_vel_(100), running_(true) {
        this->declare_parameter("k", 0.13);
        this->declare_parameter("base_vel", 50);
            
        k_ = this->get_parameter("k").as_double();
        base_vel_ = this->get_parameter("base_vel").as_int();
        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));  // Reliable (dxl_wsl과 동일)

        raw_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "Image_Topic", sub_qos,
            std::bind(&LineTrackerProcessor::image_callback, this, std::placeholders::_1));

        vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", pub_qos);

        // 초기 추적 위치 설정
        last_line_x_ = 320.0;
        last_line_y_ = 45.0;
        
        RCLCPP_INFO(this->get_logger(), "분석 및 제어 노드 시작. 's': 주행, 'q': 정지");

        key_thread_ = std::thread(&LineTrackerProcessor::keyboardLoop, this);
    }

    ~LineTrackerProcessor() {
        running_ = false;
        if (key_thread_.joinable()) key_thread_.join();
    }

private:
    // --- 1. 전처리 함수 (ROI 설정 및 이진화) ---
    cv::Mat setROI(cv::Mat &frame) {
        // 하단 1/4 영역 추출
        cv::Rect roi_rect(0, frame.rows * 3 / 4, frame.cols, frame.rows / 4);
        cv::Mat roi = frame(roi_rect).clone();

        
        cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
        // ROI 내부 평균 밝기를 이용한 보정
        roi += cv::Scalar(100) - cv::mean(roi); 
        cv::threshold(roi, roi, 150, 255, cv::THRESH_BINARY);
        
        return roi;
    }

    // --- 2. 라인 탐색 함수 (가장 가까운 객체 index 반환) ---
    int findLine(cv::Mat &bin_roi, cv::Mat &stats, cv::Mat &centroids) {
        int n_labels = cv::connectedComponentsWithStats(bin_roi, labels_, stats, centroids);

        // [1단계] 현재 중심점과 가장 가까운 후보 찾기
        int min_index = -1;
        double min_dist = static_cast<double>(bin_roi.cols);

        for (int i = 1; i < n_labels; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area > 100) { // 면적 50 이상만 취급
                double cx = centroids.at<double>(i, 0);
                double cy = centroids.at<double>(i, 1);
                double dist = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_line_x_, last_line_y_));

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

        // 최종 거리가 30px 이내일 때만 유효한 인덱스로 인정
        if (best > 30.0) {
            idx = -1;
        }
        return idx;
    }

    // --- 3. 시각화 함수 ---
    cv::Mat drawResult(cv::Mat &bin_roi, cv::Mat &stats, int best_idx) {
        cv::Mat display;
        cv::cvtColor(bin_roi, display, cv::COLOR_GRAY2BGR);

        for (int i = 1; i < stats.rows; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area < 100) continue;

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
        auto startTime = std::chrono::steady_clock::now();
        
        k_ = this->get_parameter("k").as_double();
        base_vel_ = this->get_parameter("base_vel").as_int();



        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        // 전처리
        cv::Mat bin_roi = setROI(frame);
        cv::Mat stats, centroids;
        // 라인 검출
        int best_idx = findLine(bin_roi, stats, centroids);
        // 
        cv::Mat display = drawResult(bin_roi, stats, best_idx);

        
        // 오차 계산 및 발행
        double error = (bin_roi.cols / 2.0) - last_line_x_; 
        geometry_msgs::msg::Vector3 vel_msg;
        if (mode_) {
            vel_msg.x = base_vel_ - error * k_;
            vel_msg.y = -(base_vel_ + error * k_);
        } else {
            vel_msg.x = 0; vel_msg.y = 0;
        }
        vel_pub_->publish(vel_msg);

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
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        while (running_) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);
            struct timeval tv = {0, 100000}; // 100ms timeout
            if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
                char ch = getchar();
                if (ch == 'q') { mode_ = false; RCLCPP_WARN(this->get_logger(), "STOP"); }
                else if (ch == 's') { mode_ = true; RCLCPP_INFO(this->get_logger(), "START"); }
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }


    // 멤버 변수
    double last_line_x_, last_line_y_;
    std::atomic<bool> mode_;
    double k_;
    int base_vel_;
    cv::Mat labels_;
    std::atomic<bool> running_;
    std::thread key_thread_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr raw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vel_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineTrackerProcessor>());
    rclcpp::shutdown();
    return 0;
}