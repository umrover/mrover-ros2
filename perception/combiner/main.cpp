#include "tag_detector.hpp"

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::Combiner>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
