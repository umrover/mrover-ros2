#include <keyboard_typing/keyboard_typing.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto keyboardT = std::make_shared<mrover::KeyboardTypingNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(keyboardT);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
