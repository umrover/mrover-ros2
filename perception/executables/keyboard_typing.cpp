#include <keyboard_typing/keyboard_typing.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

auto main(int argc, char* argv[]) -> int {
    rclcpp::init(argc, argv);
    auto imgKD = std::make_shared<mrover::KeyboardTypingNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(imgKD);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
