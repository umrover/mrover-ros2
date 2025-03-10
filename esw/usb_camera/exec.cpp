#include "usb_camera.hpp"

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::UsbCamera>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
