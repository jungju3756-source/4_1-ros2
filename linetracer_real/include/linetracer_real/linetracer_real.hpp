#ifndef LINETRACER_REAL_HPP_
#define LINETRACER_REAL_HPP_

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
    LineTrackerProcessor();
    ~LineTrackerProcessor();

private:
    cv::Mat setROI(cv::Mat &frame);
    int findLine(cv::Mat &bin_roi, cv::Mat &stats, cv::Mat &centroids);
    cv::Mat drawResult(cv::Mat &bin_roi, cv::Mat &stats, int best_idx);
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void keyboardLoop();

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

#endif // LINETRACER_REAL_HPP_