#include <cost_map/cost_map.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::CostMapNode>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
