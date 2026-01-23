#include "gst_websocket_streamer.hpp"

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::GstWebsocketStreamer>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
