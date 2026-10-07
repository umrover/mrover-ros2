#include <gesture_detection/gesture_detection.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto gestureR = std::make_shared<mrover::GestureRecognitionNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(gestureR);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}