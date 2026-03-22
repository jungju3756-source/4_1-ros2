#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <chrono>

// Raspberry Pi 5 libcamera GStreamer pipeline
static const std::string GST_SRC =
    "libcamerasrc ! "
    "video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "
    "queue ! videoconvert ! videoflip method=rotate-180 ! "
    "videoconvert ! video/x-raw,format=(string)BGR ! appsink";

class CameraPublisher : public rclcpp::Node {
public:
    CameraPublisher() : Node("camera_publisher_node") {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        raw_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("Image_Topic", qos_profile);

        cap_.open(GST_SRC, cv::CAP_GSTREAMER);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera!");
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&CameraPublisher::timer_callback, this));
    }

private:
    void timer_callback() {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame from camera");
            return;
        }
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toCompressedImageMsg();
        raw_image_pub_->publish(*msg);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr raw_image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
