#include <key_model/key_detector.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto imgKD = std::make_shared<mrover::ImageKeyDetector>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(imgKD);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
