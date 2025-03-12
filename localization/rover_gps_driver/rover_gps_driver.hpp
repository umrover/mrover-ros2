#pragma once

#include "mrover/msg/detail/gps_velocity__struct.hpp"
#include "pch.hpp"
#include <geometry_msgs/msg/detail/vector3_stamped__struct.hpp>

namespace mrover {

    class RoverGPSDriver : public rclcpp::Node {

    private:
        void process_rtcm(const rtcm_msgs::msg::Message::ConstSharedPtr &rtcm_msg);
        void process_unicore(std::string &unicore_msg);

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;
        rclcpp::Publisher<mrover::msg::FixStatus>::SharedPtr gps_status_pub;
        rclcpp::Publisher<mrover::msg::Heading>::SharedPtr heading_pub;
        rclcpp::Publisher<mrover::msg::FixStatus>::SharedPtr heading_status_pub;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_pub;
        rclcpp::Publisher<mrover::msg::FixStatus>::SharedPtr velocity_status_pub;
        rclcpp::Publisher<mrover::msg::SatelliteSignal>::SharedPtr satellite_signal_pub;
        rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub;

        boost::asio::basic_serial_port<boost::asio::io_context::executor_type> serial;
        boost::asio::streambuf read_buffer;

        unsigned long baud;
        std::string port;
        std::string frame_id;

        constexpr static std::string GNGGA_HEADER = "$GNGGA";
        constexpr static std::string UNIHEADING_HEADER = "#UNIHEADINGA";
        constexpr static std::string BESTNAV_HEADER = "#BESTNAVA";

        constexpr static std::string GPS_HEADER = "$GPGSV";
        constexpr static std::string GLONASS_HEADER = "$GLGSV";
        constexpr static std::string BEIDOU_HEADER = "$GBGSV";
        constexpr static std::string GALILEO_HEADER = "$GAGSV";
        constexpr static std::string QZSS_HEADER = "$GQGSV";

        constexpr static uint8_t GNGGA_LAT_POS = 2;
        constexpr static uint8_t GNGGA_LAT_DIR_POS = 3;
        constexpr static uint8_t GNGGA_LON_POS = 4;
        constexpr static uint8_t GNGGA_LON_DIR_POS = 5;
        constexpr static uint8_t GNGGA_QUAL_POS = 6;
        constexpr static uint8_t GNGGA_SATELLITES_POS = 7;
        constexpr static uint8_t GNGGA_ALT_POS = 9;

        constexpr static uint8_t UNIHEADING_STATUS_POS = 11;
        constexpr static uint8_t UNIHEADING_HEADING_POS = 13;

        constexpr static uint8_t VEL_STATUS_POS = 32;
        constexpr static uint8_t VEL_POS = 35;
        constexpr static uint8_t VEL_DIR_POS = 36;


        constexpr static uint8_t CNO_POS = 7;
        

    public:
        explicit RoverGPSDriver(boost::asio::io_context& io);
        void spin();

    }; // class RoverDriverNode
} // namespace mrover

