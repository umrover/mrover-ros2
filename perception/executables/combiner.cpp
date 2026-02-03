#include <combiner/combiner.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::Combiner>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
