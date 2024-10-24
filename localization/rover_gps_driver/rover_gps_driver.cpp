#include "rover_gps_driver.hpp"

namespace mrover {

    RoverGPSDriver::RoverGPSDriver(boost::asio::io_context& io) : Node("rover_gps_driver"), serial(io) {
        
        // connect to serial
        declare_parameter("port", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("baud", rclcpp::ParameterType::PARAMETER_INTEGER);

        port = get_parameter("port").as_string();
        baud = get_parameter("baud").as_int();

        serial.open(port);
        std::string port_string = std::to_string(baud);
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud));

        RCLCPP_INFO(get_logger(), "Connected to GPS via serial!");
        
        // publishers and subscribers
        gps_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
        rtk_status_pub = this->create_publisher<mrover::msg::RTKStatus>("/rtk_fix_status", 10);
        rtcm_sub = this->create_subscription<rtcm_msgs::msg::Message>("/rtcm", 10, [&](rtcm_msgs::msg::Message::ConstSharedPtr const& rtcm_message) {
            process_rtcm(rtcm_message);
        });

    }

    void RoverGPSDriver::process_rtcm(rtcm_msgs::msg::Message::ConstSharedPtr rtcm_msg) {
        boost::asio::write(serial, boost::asio::buffer(rtcm_msg->message));
    }

    // just want to log the messages for now
    void RoverGPSDriver::process_unicore(std::string unicore_msg) {
        RCLCPP_INFO(get_logger(), ("Message: " + unicore_msg).c_str());

        // parse messages here:
    }

    void RoverGPSDriver::spin() {
        while (rclcpp::ok()) {
            boost::asio::read_until(serial, read_buffer, '\n');
            std::istream buffer(&read_buffer);
            std::string unicore_msg;
            std::getline(buffer, unicore_msg);
            process_unicore(unicore_msg);
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

}

int main(int argc, char**argv) {
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
    rclcpp::init(argc, argv);

    boost::asio::io_context io;
    auto node = std::make_shared<mrover::RoverGPSDriver>(io);

    node->spin();
    rclcpp::shutdown();

    return 0;
}
