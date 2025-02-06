#include <object_detector/object_detector.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto imgOD = std::make_shared<mrover::ImageObjectDetector>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(imgOD);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
