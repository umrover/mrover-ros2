#include "key_detector.hpp"

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    // DO NOT REMOVE OR ELSE REF COUNT WILL GO TO ZERO
    rclcpp::executors::SingleThreadedExecutor executor;
        
    auto actionServer = std::make_shared<mrover::KeyDetector>();
    rclcpp::executors::SingleThreadedExecutor server;


    server.add_node(actionServer);
    server.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
