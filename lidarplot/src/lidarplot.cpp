#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)

class LidarPlotNode : public rclcpp::Node
{
public:
    LidarPlotNode() : Node("lidarplot_node")
    {
        // /scan 토픽 구독
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarPlotNode::scanCb, this, std::placeholders::_1));

        // 동영상 저장을 위한 VideoWriter 초기화 (크기: 500x500, FPS: 10, 코덱: mp4v)
        video_writer_.open("lidar_scan_output.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 10.0, cv::Size(500, 500));
        
        RCLCPP_INFO(this->get_logger(), "LidarPlot Node has been started. Subscribing to /scan");
    }

    ~LidarPlotNode()
    {
        if (video_writer_.isOpened()) {
            video_writer_.release();
        }
        cv::destroyAllWindows();
    }

private:
    void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        int count = scan->scan_time / scan->time_increment;
        if (count > static_cast<int>(scan->ranges.size())) {
            count = scan->ranges.size(); // 예외처리: 실제 배열 크기를 초과하지 않도록 보정
        }

        printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
        printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

        // 1. 500x500 흰색 이미지 생성 (1채널은 CV_8UC1이지만 컴파일을 위해 CV_8UC3으로 컬러 이미지 사용)
        cv::Mat frame(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
        
        // 2. 중심(250, 250)에 라이다 위치 (십자가) 표시
        cv::line(frame, cv::Point(250, 240), cv::Point(250, 260), cv::Scalar(0, 0, 0), 2);
        cv::line(frame, cv::Point(240, 250), cv::Point(260, 250), cv::Scalar(0, 0, 0), 2);

        for (int i = 0; i < count; i++) {
            float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            float range = scan->ranges[i];
            
            // 실시간 동작 시 터미널 로그 과부하를 방지하기 위해 각도/거리 출력 콘솔 로그는 주석 처리합니다.
            // printf("[SLLIDAR INFO]: angle-distance : [%f, %f]\n", degree, range);

            // 3. 스캔영상 그리기 코드 추가
            // 거리(range)가 무한대가 아니고 0.0보다 큰 유효한 값일 경우에만 그림
            if (!std::isinf(range) && !std::isnan(range) && range > 0.0) {
                // 10m를 500픽셀로 매핑 (1m당 50픽셀)
                float r_pixel = range * 50.0;
                
                // 현재 인덱스의 라디안 단위 각도
                float angle_rad = scan->angle_min + scan->angle_increment * i;
                
                // 첨부된 강의/슬라이드 자료에 나온 수식 적용 
                // x = 250 + R*sin(PI - theta), y = 250 - R*cos(PI - theta)
                int x = 250 + r_pixel * sin(M_PI - angle_rad);
                int y = 250 - r_pixel * cos(M_PI - angle_rad);
                
                // 500x500 해상도 안에 들어오는 장애물만 빨간색 점(크기 2)으로 표시
                if (x >= 0 && x < 500 && y >= 0 && y < 500) {
                    cv::circle(frame, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
                }
            }
        }
        
        // 4. 스캔영상 화면출력 및 동영상 저장코드 추가
        cv::imshow("Lidar Scan", frame);
        cv::waitKey(1); // 영상 갱신을 위해 1ms 대기
        
        if (video_writer_.isOpened()) {
            video_writer_.write(frame);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    cv::VideoWriter video_writer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPlotNode>());
    rclcpp::shutdown();
    return 0;
}
