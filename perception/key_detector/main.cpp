#include "key_detector.hpp"

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    // DO NOT REMOVE OR ELSE REF COUNT WILL GO TO ZERO
	auto keyDetector = std::make_shared<mrover::KeyDetector>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(keyDetector);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
