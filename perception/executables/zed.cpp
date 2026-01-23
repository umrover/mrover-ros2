#include <zed_wrapper/zed_wrapper.hpp>

auto main(int argc, char* argv[]) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::ZedWrapper>());
    rclcpp::shutdown();
    return 0;
}
