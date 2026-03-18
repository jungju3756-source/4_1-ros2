#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>

using std::placeholders::_1;

class DxlSubscriber : public rclcpp::Node
{
public:
    DxlSubscriber()
        : Node("dxl_subscriber")
    {
        sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "topic_dxlpub", 10,
            std::bind(&DxlSubscriber::cmd_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "DxlSubscriber 시작 | /topic_dxlpub 대기 중...");
    }

private:
    void cmd_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
            "[모터 시뮬] left_rpm: %.1f | right_rpm: %.1f",
            msg->x, msg->y);
    }

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DxlSubscriber>());
    rclcpp::shutdown();
    return 0;
}
