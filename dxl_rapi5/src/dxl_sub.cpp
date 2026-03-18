#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

using std::placeholders::_1;

// ── Control Table (Protocol 2.0, XM/XC 시리즈) ─────────────────────────────
static constexpr uint16_t ADDR_OPERATING_MODE = 11;
static constexpr uint16_t ADDR_TORQUE_ENABLE  = 64;
static constexpr uint16_t ADDR_GOAL_VELOCITY  = 104;
static constexpr uint8_t  OPERATING_MODE_VEL  = 1;   // 속도 제어 모드
static constexpr uint8_t  TORQUE_ENABLE       = 1;
static constexpr uint8_t  TORQUE_DISABLE      = 0;
static constexpr double   RPM_TO_UNIT         = 1.0 / 0.229; // 1rpm = 4.37 unit

class DxlSub : public rclcpp::Node
{
public:
    DxlSub()
        : Node("dxl_sub")
    {
        // 파라미터 선언
        this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate",  57600);
        this->declare_parameter<int>("left_id",   1);
        this->declare_parameter<int>("right_id",  2);

        std::string device = this->get_parameter("device_name").as_string();
        int baudrate        = this->get_parameter("baudrate").as_int();
        left_id_            = (uint8_t)this->get_parameter("left_id").as_int();
        right_id_           = (uint8_t)this->get_parameter("right_id").as_int();

        // PortHandler / PacketHandler 초기화
        port_handler_   = dynamixel::PortHandler::getPortHandler(device.c_str());
        packet_handler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

        if (!port_handler_->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "포트 열기 실패: %s", device.c_str());
            return;
        }
        if (!port_handler_->setBaudRate(baudrate)) {
            RCLCPP_ERROR(this->get_logger(), "Baudrate 설정 실패: %d", baudrate);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "포트 열기 성공: %s @ %d", device.c_str(), baudrate);

        setup_motor(left_id_,  "Left");
        setup_motor(right_id_, "Right");

        sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "topic_dxlpub", 10,
            std::bind(&DxlSub::cmd_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "DxlSub 시작 | /topic_dxlpub 대기 중...");
    }

    ~DxlSub()
    {
        // 종료 시 모터 정지 및 토크 비활성화
        set_velocity(left_id_,  0);
        set_velocity(right_id_, 0);
        packet_handler_->write1ByteTxRx(port_handler_, left_id_,  ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);
        packet_handler_->write1ByteTxRx(port_handler_, right_id_, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);
        port_handler_->closePort();
    }

private:
    void setup_motor(uint8_t id, const char* name)
    {
        uint8_t dxl_error = 0;
        int result;

        // 토크 OFF → 모드 설정 → 토크 ON
        result = packet_handler_->write1ByteTxRx(
            port_handler_, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
        if (result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "[%s ID:%d] 토크 비활성화 실패", name, id);
            return;
        }

        result = packet_handler_->write1ByteTxRx(
            port_handler_, id, ADDR_OPERATING_MODE, OPERATING_MODE_VEL, &dxl_error);
        if (result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "[%s ID:%d] 속도 모드 설정 실패", name, id);
            return;
        }

        result = packet_handler_->write1ByteTxRx(
            port_handler_, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "[%s ID:%d] 토크 활성화 실패", name, id);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "[%s ID:%d] 초기화 완료", name, id);
    }

    void set_velocity(uint8_t id, int32_t value)
    {
        uint8_t dxl_error = 0;
        packet_handler_->write4ByteTxRx(
            port_handler_, id, ADDR_GOAL_VELOCITY, (uint32_t)value, &dxl_error);
    }

    void cmd_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        // rpm → Dynamixel 단위 변환
        // 오른쪽 모터는 반대 방향 장착 → 부호 반전
        int32_t left_val  = (int32_t)( msg->x * RPM_TO_UNIT);
        int32_t right_val = (int32_t)(-msg->y * RPM_TO_UNIT);

        set_velocity(left_id_,  left_val);
        set_velocity(right_id_, right_val);

        RCLCPP_INFO(this->get_logger(),
            "left_rpm:%.1f(unit:%d) | right_rpm:%.1f(unit:%d)",
            msg->x, left_val, msg->y, right_val);
    }

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_;

    dynamixel::PortHandler   * port_handler_;
    dynamixel::PacketHandler * packet_handler_;

    uint8_t left_id_;
    uint8_t right_id_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DxlSub>());
    rclcpp::shutdown();
    return 0;
}
