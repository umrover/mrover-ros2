#include "key_detector.hpp"
#include "key_loc_server_placeholder.hpp"

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    // DO NOT REMOVE OR ELSE REF COUNT WILL GO TO ZERO
    rclcpp::executors::SingleThreadedExecutor executor;

    auto actionServer = std::make_shared<mrover::KeyDetector>();
    rclcpp::executors::SingleThreadedExecutor server;

    server.add_node(actionServer);
    //server.add_node(std::make_shared<GetKeyLocServer>());
    server.spin();

    rclcpp::shutdown();

    return EXIT_SUCCESS;
}
