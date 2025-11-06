#include "click_ik.hpp"
#include <memory>
#include <rclcpp/utilities.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::ClickIkNode>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}