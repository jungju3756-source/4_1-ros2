#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher()
        : Node("video_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video", 10);

        // 동영상 경로 설정
        cap.open("/home/aim88/simulation/7_lt_ccw_100rpm_in.mp4");

        if(!cap.isOpened()){
            RCLCPP_ERROR(this->get_logger(), "!!! 영상 파일을 열 수 없습니다 !!!");
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
        cap >> frame;

        if(frame.empty()) {
            RCLCPP_INFO(this->get_logger(), "영상 재시작");
            cap.set(cv::CAP_PROP_POS_FRAMES, 0);
            return;
        }

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "이미지 송신 중...");
        publisher_->publish(*msg);
    }

    cv::VideoCapture cap;
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

