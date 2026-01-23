#include <tag_detector/tag_detector.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto imageTD = std::make_shared<mrover::ImageTagDetector>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(imageTD);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
