#include "tag_detector.hpp"

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    auto stereoTD = std::make_shared<mrover::StereoTagDetectorNodelet>();
    auto imageTD = std::make_shared<mrover::ImageTagDetectorNodelet>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(stereoTD);
    executor.add_node(imageTD);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
