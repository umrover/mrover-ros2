#pragma once

#include "pch.hpp"

namespace mrover {
    class RoverDriverNode : public rclcpp::Node {
    private:
        void process_rtcm(Message rtcm_message);
        void process_unicore(std::string unicore_message);

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;
        rclcppp::Publisher<mrover::msg::RTKStatus>::SharedPtr rtk_status_pub;
        rclcpp::Subscriber<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub;

        const unsigned short baud;
        const std::string port;



    public:
        RoverDriverNode();
    }; // class RoverDriverNode
} // namespace mrover

