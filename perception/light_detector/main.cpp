#include "light_detector.hpp"

auto main(int argc, char* argv[]) -> int{
    rclcpp::init(argc, argv);

    // DO NOT REMOVE OR ELSE REF COUNT WILL GO TO ZERO
    auto lightOD = std::make_shared<mrover::ColoredDetector>();
    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(lightOD);

    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
