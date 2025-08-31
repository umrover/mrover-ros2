#pragma once

#include "pch.hpp"

namespace mrover {

    class RoverGPSDriver : public rclcpp::Node {

    private:
        void process_rtcm(const rtcm_msgs::msg::Message::ConstSharedPtr &rtcm_msg);
        void process_unicore(std::string &unicore_msg);

        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub;
        rclcpp::Publisher<mrover::msg::FixStatus>::SharedPtr gps_status_pub;
        rclcpp::Publisher<mrover::msg::SatelliteSignal>::SharedPtr satellite_signal_pub;
        rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub;

        boost::asio::basic_serial_port<boost::asio::io_context::executor_type> serial;
        boost::asio::streambuf read_buffer;

        unsigned long baud;
        std::string port;
        std::string gps_frame;

        // data store
        std::map<std::string, std::string> headers_to_constellations;
        std::map<std::string, std::vector<uint8_t> > satellite_signals;

        const uint8_t MAX_SATELLITE_SIGNAL_SIZE = 36;

        // reference coordinates
        double ref_lat;
        double ref_lon;
        double ref_alt;
        

    public:
        explicit RoverGPSDriver(boost::asio::io_context& io);
        void spin();
        void stop();

    }; // class RoverDriverNode
} // namespace mrover

