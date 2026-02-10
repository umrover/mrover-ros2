#include <zed_timer/zed_timer.hpp>
#include <rclcpp/executors.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::ZedTimerNode>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}