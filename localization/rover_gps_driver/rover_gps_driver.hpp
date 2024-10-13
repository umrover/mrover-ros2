#pragma once

#include "pch.hpp"

namespace mrover {

    class RoverGPSDriver : public rclcpp::Node {

    private:
        void process_rtcm(rtcm_msgs::msg::Message::ConstSharedPtr rtcm_msg);
        void process_unicore(std::string unicore_msg);

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;
        rclcpp::Publisher<mrover::msg::RTKStatus>::SharedPtr rtk_status_pub;
        rclcpp::Subscriber<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub;

        boost::asio::basic_serial_port<boost::asio::io_context::executor_type> serial;

        unsigned short baud;
        std::string port;

    public:
        explicit RoverGPSDriver(boost::asio::io_context& io);
        void spin();

    }; // class RoverDriverNode
} // namespace mrover

