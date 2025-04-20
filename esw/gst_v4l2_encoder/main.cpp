#include "gst_v4l2_encoder.hpp"

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::GstV4L2Encoder>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
