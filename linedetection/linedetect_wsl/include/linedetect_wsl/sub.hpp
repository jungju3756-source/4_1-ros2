#ifndef _SUB_HPP_
#define _SUB_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
#include <chrono>
using std::placeholders::_1; // std::bind 사용을 위한 선언
using namespace std;
using namespace cv;

class LineDetector : public rclcpp::Node { 
    private: 
    void mysub_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    Mat labels, stats, centroids;//stats: 각 레이블의 정보(x,y,width,height) 
                                //centroids: 각 레이블의 무게중심(x좌표,y좌표) 
    Point2f past_point, present_point;//past_point = 과거 좌표값, present_point = 현재 좌표값 
    int cnt; 
    double distance,target_x,min_dist,best_idx; 
    Point tmp_pt_;
    bool first_run_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_; 
    public: LineDetector(); 
};
#endif