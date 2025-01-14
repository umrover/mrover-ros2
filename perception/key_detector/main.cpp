#include "key_detector.hpp"

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    // DO NOT REMOVE OR ELSE REF COUNT WILL GO TO ZERO
	auto keyDetector = std::make_shared<mrover::KeyDetector>();
    auto actionServer = std::make_shared<mrover::KeyActionServer>();

    rclcpp::executors::SingleThreadedExecutor executor;

    rclcpp::executors::SingleThreadedExecutor server;
    
    executor.add_node(keyDetector);
    executor.spin();

    server.add_node(actionServer);
    server.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
