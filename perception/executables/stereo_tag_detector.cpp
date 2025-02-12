#include <tag_detector/tag_detector.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto stereoTD = std::make_shared<mrover::StereoTagDetectorNodelet>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(stereoTD);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
