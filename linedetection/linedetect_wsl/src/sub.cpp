#include "linedetect_wsl/sub.hpp"
#include <memory> 
#include <chrono> 

using namespace std;
using std::placeholders::_1;

// 생성자
LineDetector::LineDetector() : Node("line_detector"), tmp_pt_(320, 60), first_run_(true) { 
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    // /video 토픽 구독 (linedetect_nano pub과 일치)
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "video", 
        qos_profile, 
        bind(&LineDetector::mysub_callback, this, _1));
}

void LineDetector::mysub_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    auto startTime = chrono::steady_clock::now();

    // cv_bridge로 ROS Image → cv::Mat 변환
    cv::Mat frame_color;
    try {
        frame_color = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge 변환 오류: %s", e.what());
        return;
    }
    if(frame_color.empty()) return;

    cv::Mat frame_gray;
    cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);       // BGR → Gray
    cv::Scalar bright_avg = cv::mean(frame_gray);                    // 밝기 평균
    frame_gray = frame_gray + (100 - bright_avg[0]);                 // 밝기 보정
    cv::Mat frame_binary;
    cv::threshold(frame_gray, frame_binary, 130, 255, cv::THRESH_BINARY); // 이진화
    cv::Mat roi = frame_binary(cv::Rect(0, 240, 640, 120));          // ROI 영역 (하단 1/4)

    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);
    int cnt = stats.rows;
    int target_idx = -1;
    
    // 라인 탐색 로직 (첫 실행: 전체 ROI 탐색 / 이후: 60px 반경 내 탐색)
    int min_idx = -1;
    int min_dist = 10000;
    int search_radius = first_run_ ? roi.cols : 60;
    cv::Point search_center = first_run_ ? cv::Point(320, 60) : tmp_pt_; 

    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 100) {
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            int dist = cv::norm(cv::Point(x, y) - search_center);
            if (dist < min_dist && dist <= search_radius) {
                min_dist = dist;
                min_idx = i;
            }
        }
    }

    if (min_idx != -1) {
        tmp_pt_ = cv::Point(cvRound(centroids.at<double>(min_idx, 0)), cvRound(centroids.at<double>(min_idx, 1)));
        target_idx = min_idx;
        first_run_ = false;
    }
    
    // 시각화 (타겟=빨간색, 나머지=파란색)
    cv::Mat result_view = roi.clone();
    cv::cvtColor(result_view, result_view, cv::COLOR_GRAY2BGR);
    
    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 100) {
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            int left   = stats.at<int>(i, 0);
            int top    = stats.at<int>(i, 1);
            int width  = stats.at<int>(i, 2);
            int height = stats.at<int>(i, 3);
            cv::Scalar color = (i == target_idx) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            cv::rectangle(result_view, cv::Rect(left, top, width, height), color, 1);
            cv::circle(result_view, cv::Point(x, y), 5, color, -1);
        }
    }

    int error = 320 - tmp_pt_.x;

    auto endTime = chrono::steady_clock::now();
    float totalTime = chrono::duration<float, milli>(endTime - startTime).count();

    RCLCPP_INFO(this->get_logger(), "이미지 수신 중 | err:%d, time:%.2f ms", error, totalTime);

    cv::imshow("frame_color", frame_color);
    cv::imshow("frame_roi", result_view);
    cv::waitKey(1);
}