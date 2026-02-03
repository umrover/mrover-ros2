#include <usb_camera/usb_camera.hpp>

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto usbCamera = std::make_shared<mrover::UsbCamera>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(usbCamera);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
