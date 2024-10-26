#pragma once

#include "pch.hpp"

namespace mrover {

    class RoverGPSDriver : public rclcpp::Node {

    private:
        void process_rtcm(rtcm_msgs::msg::Message::ConstSharedPtr rtcm_msg);
        void process_unicore(std::string unicore_msg);

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;
        rclcpp::Publisher<mrover::msg::RTKStatus>::SharedPtr gps_status_pub;
        rclcpp::Publisher<mrover::msg::RTKHeading>::SharedPtr heading_pub;
        rclcpp::Publisher<mrover::msg::RTKStatus>::SharedPtr heading_status_pub;
        rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub;

        boost::asio::basic_serial_port<boost::asio::io_context::executor_type> serial;
        boost::asio::streambuf read_buffer;

        unsigned long baud;
        std::string port;
        std::string frame_id;

        constexpr static std::string GNGGA_HEADER = "$GNGGA";
        constexpr static std::string ADRNAV_HEADER = "#ADRNAVA";
        constexpr static std::string UNIHEADING_HEADER = "#UNIHEADINGA";

        constexpr static uint8_t GNGGA_LAT_POS = 2;
        constexpr static uint8_t GNGGA_LAT_DIR_POS = 3;
        constexpr static uint8_t GNGGA_LON_POS = 4;
        constexpr static uint8_t GNGGA_LON_DIR_POS = 5;
        constexpr static uint8_t GNGGA_ALT_POS = 9;

        constexpr static uint8_t ADRNAV_LAT_POS = 12;
        constexpr static uint8_t ADRNAV_LON_POS = 13;
        constexpr static uint8_t ADRNAV_ALT_POS = 14;
        constexpr static uint8_t ADRNAV_STATUS_POS = 10;

        constexpr static uint8_t UNIHEADING_HEADING_POS = 13;
        constexpr static uint8_t UNIHEADING_STATUS_POS = 10;
        

    public:
        explicit RoverGPSDriver(boost::asio::io_context& io);
        void spin();

    }; // class RoverDriverNode
} // namespace mrover

