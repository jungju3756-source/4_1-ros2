#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher()
        : Node("video_publisher")
    {
        this->declare_parameter<std::string>("video_path", "5_lt_cw_100rpm_out.mp4");
        std::string video_path = this->get_parameter("video_path").as_string();

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video", 10);

        cap_.open(video_path);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "!!! 영상 파일을 열 수 없습니다: %s !!!", video_path.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "영상 파일 열기 성공: %s", video_path.c_str());
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(30),
            std::bind(&VideoPublisher::timer_callback, this)
        );
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "영상 재시작");
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "이미지 송신 중...");
        publisher_->publish(*msg);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
}
