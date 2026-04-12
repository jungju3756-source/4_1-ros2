#include "rclcpp/rclcpp.hpp"
#include "linetracer_real/linetracer_real.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineTrackerProcessor>());
    rclcpp::shutdown();
    return 0;
}