#include "object_detector.hpp"

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);

    // DO NOT REMOVE OR ELSE REF COUNT WILL GO TO ZERO
    auto imgOD = std::make_shared<mrover::ImageObjectDetector>();
    auto stereoOD = std::make_shared<mrover::StereoObjectDetector>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(imgOD);
    executor.add_node(stereoOD);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
