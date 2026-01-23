#include <object_detector/object_detector.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto stereoOD = std::make_shared<mrover::StereoObjectDetector>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(stereoOD);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
