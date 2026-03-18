#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <chrono>

using std::placeholders::_1;

static constexpr double BASE_RPM = 100.0;
static constexpr double K        = 1.0;

class LineController : public rclcpp::Node
{
public:
    LineController()
        : Node("line_controller"),
          tmp_pt_(320, 60),
          first_run_(true),
          send_commands_(false)
    {
        // 터미널 raw mode
        tcgetattr(STDIN_FILENO, &orig_termios_);
        struct termios raw = orig_termios_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "video", qos,
            std::bind(&LineController::image_callback, this, _1));

        pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", 10);

        RCLCPP_INFO(this->get_logger(), "LineController 시작 | s: 전송시작, q: 정지");
    }

    ~LineController()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
        cv::destroyAllWindows();
    }

private:
    void poll_keyboard()
    {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        struct timeval tv{0, 0};

        if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
            char c;
            if (read(STDIN_FILENO, &c, 1) == 1) {
                if (c == 's') {
                    send_commands_ = true;
                    RCLCPP_INFO(this->get_logger(), ">>> 속도 명령 전송 시작 (s)");
                } else if (c == 'q') {
                    send_commands_ = false;
                    geometry_msgs::msg::Vector3 stop_msg;
                    stop_msg.x = 0.0;
                    stop_msg.y = 0.0;
                    pub_->publish(stop_msg);
                    RCLCPP_INFO(this->get_logger(), ">>> 정지 명령 전송 (q)");
                }
            }
        }
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        poll_keyboard();

        auto start = std::chrono::steady_clock::now();

        cv::Mat frame_color;
        try {
            frame_color = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 오류: %s", e.what());
            return;
        }
        if (frame_color.empty()) return;

        // 검출 파이프라인
        cv::Mat frame_gray;
        cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);
        cv::Scalar bright_avg = cv::mean(frame_gray);
        frame_gray = frame_gray + (100 - bright_avg[0]);

        cv::Mat frame_binary;
        cv::threshold(frame_gray, frame_binary, 130, 255, cv::THRESH_BINARY);

        cv::Mat roi = frame_binary(cv::Rect(0, 240, 640, 120));

        cv::Mat labels, stats, centroids;
        cv::connectedComponentsWithStats(roi, labels, stats, centroids);
        int cnt = stats.rows;

        int min_idx  = -1;
        int min_dist = 10000;
        int search_radius   = first_run_ ? roi.cols : 60;
        cv::Point search_center = first_run_ ? cv::Point(320, 60) : tmp_pt_;

        for (int i = 1; i < cnt; i++) {
            int area = stats.at<int>(i, 4);
            if (area > 100) {
                int x = cvRound(centroids.at<double>(i, 0));
                int y = cvRound(centroids.at<double>(i, 1));
                int dist = (int)cv::norm(cv::Point(x, y) - search_center);
                if (dist < min_dist && dist <= search_radius) {
                    min_dist = dist;
                    min_idx  = i;
                }
            }
        }

        if (min_idx != -1) {
            tmp_pt_   = cv::Point(cvRound(centroids.at<double>(min_idx, 0)),
                                  cvRound(centroids.at<double>(min_idx, 1)));
            first_run_ = false;
        }

        // 시각화
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
                cv::Scalar color = (i == min_idx) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
                cv::rectangle(result_view, cv::Rect(left, top, width, height), color, 1);
                cv::circle(result_view, cv::Point(x, y), 5, color, -1);
            }
        }

        int error      = 320 - tmp_pt_.x;
        double left_rpm  = BASE_RPM - K * error;
        double right_rpm = BASE_RPM + K * error;

        auto end = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float, std::milli>(end - start).count();

        RCLCPP_INFO(this->get_logger(),
            "err:%d | left:%.1f | right:%.1f | time:%.2fms | sending:%s",
            error, left_rpm, right_rpm, elapsed,
            send_commands_ ? "YES" : "NO");

        if (send_commands_) {
            geometry_msgs::msg::Vector3 cmd;
            cmd.x = left_rpm;
            cmd.y = right_rpm;
            pub_->publish(cmd);
        }

        cv::imshow("frame_color", frame_color);
        cv::imshow("frame_roi", result_view);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;

    cv::Point tmp_pt_;
    bool first_run_;
    bool send_commands_;

    struct termios orig_termios_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineController>());
    rclcpp::shutdown();
    return 0;
}
