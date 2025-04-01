#include <lander_analysis/lander_analysis.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto landerAnalysis = std::make_shared<mrover::LanderAnalysis>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(landerAnalysis);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
