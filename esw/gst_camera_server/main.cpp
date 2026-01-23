#include "gst_camera_server.hpp"

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::GstCameraServer>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
