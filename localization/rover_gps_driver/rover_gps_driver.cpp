#include "rover_gps_driver.hpp"

namespace mrover {

    RoverGPSDriver::RoverGPSDriver(boost::asio::io_context& io) : Node("rover_gps_driver"), serial(io) {
        
        // connect to serial
        declare_parameter("port", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("baud", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("frame_id", rclcpp::ParameterType::PARAMETER_STRING);

        port = get_parameter("port").as_string();
        baud = get_parameter("baud").as_int();
        frame_id = get_parameter("frame_id").as_string();

        serial.open(port);
        std::string port_string = std::to_string(baud);
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud));

        RCLCPP_INFO(get_logger(), "Connected to GPS via serial!");
        
        // publishers and subscribers
        gps_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
        gps_status_pub = this->create_publisher<mrover::msg::RTKStatus>("/gps_fix_status", 10);
        heading_pub = this->create_publisher<mrover::msg::RTKHeading>("/heading/fix", 10);
        heading_status_pub = this->create_publisher<mrover::msg::RTKStatus>("/heading_fix_status", 10);
        rtcm_sub = this->create_subscription<rtcm_msgs::msg::Message>("/rtcm", 10, [&](rtcm_msgs::msg::Message::ConstSharedPtr const& rtcm_message) {
            process_rtcm(rtcm_message);
        });
    }

    void RoverGPSDriver::process_rtcm(const rtcm_msgs::msg::Message::ConstSharedPtr &rtcm_msg) {
        boost::asio::write(serial, boost::asio::buffer(rtcm_msg->message));
    }

    void RoverGPSDriver::process_unicore(std::string &unicore_msg) {

        std::vector<std::string> tokens;
        boost::split(tokens, unicore_msg, boost::is_any_of(",;"));

        std::string msg_header = tokens[0];

        std_msgs::msg::Header header;
        header.stamp = get_clock()->now();
        header.frame_id = frame_id;

        if (msg_header == GNGGA_HEADER) {

            sensor_msgs::msg::NavSatFix nav_sat_fix;

            if (tokens[GNGGA_LAT_POS].empty() || tokens[GNGGA_LON_POS].empty() || tokens[GNGGA_ALT_POS].empty()) {
                RCLCPP_WARN(get_logger(), "No satellite fix. Are we inside?");
                return;
            }
            
            uint16_t lat_deg = stoi(tokens[GNGGA_LAT_POS].substr(0, 2));
            double lat_min = stod(tokens[GNGGA_LAT_POS].substr(2, 13));
            uint16_t lon_deg = stoi(tokens[GNGGA_LON_POS].substr(0, 3));
            double lon_min = stod(tokens[GNGGA_LON_POS].substr(3, 14));
            double alt = stod(tokens[GNGGA_ALT_POS]);

            // 60 minutes = 1 degree
            double lat = lat_deg + lat_min / 60;
            double lon = lon_deg + lon_min / 60;

            char lat_dir = tokens[GNGGA_LAT_DIR_POS][0];
            char lon_dir = tokens[GNGGA_LON_DIR_POS][0];

            if (lat_dir == 'S') {
                lat = -lat;
            }
            if (lon_dir == 'W') {
                lon = -lon;
            }

            // TODO: get real covariance
            std::array<double, 9> cov = {1, 0, 0, 0, 1, 0, 0, 0, 1};

            nav_sat_fix.header = header;
            nav_sat_fix.latitude = lat;
            nav_sat_fix.longitude = lon;
            nav_sat_fix.altitude = alt;
            nav_sat_fix.position_covariance = cov;

            gps_pub->publish(nav_sat_fix);

        }

        if (msg_header == ADRNAV_HEADER) {

            sensor_msgs::msg::NavSatFix nav_sat_fix;
            mrover::msg::RTKStatus rtk_status;
            mrover::msg::RTKFixType sol_status;

            if (tokens[ADRNAV_STATUS_POS] == "SOL_COMPUTED") {
                sol_status.fix_type = mrover::msg::RTKFixType::SOL_COMPUTED;
            }
            else if (tokens[ADRNAV_STATUS_POS] == "NO_CONVERGENCE") {
                sol_status.fix_type = mrover::msg::RTKFixType::NO_CONVERGENCE;
            }
            else if (tokens[ADRNAV_STATUS_POS] == "COV_TRACE") {
                sol_status.fix_type = mrover::msg::RTKFixType::COV_TRACE;
            }
            else {
                RCLCPP_WARN(get_logger(), "No RTK fix. Is the basestation on?");
                return;
            }

            rtk_status.sol_status = sol_status;

            double lat = stod(tokens[ADRNAV_LAT_POS]);
            double lon = stod(tokens[ADRNAV_LON_POS]);
            double alt = stod(tokens[ADRNAV_ALT_POS]);

            // TODO: get real covariance
            std::array<double, 9> cov = {1, 0, 0, 0, 1, 0, 0, 0, 1};

            nav_sat_fix.header = header;
            nav_sat_fix.latitude = lat;
            nav_sat_fix.longitude = lon;
            nav_sat_fix.altitude = alt;
            nav_sat_fix.position_covariance = cov;

            rtk_status.header = header;

            gps_pub->publish(nav_sat_fix);
            gps_status_pub->publish(rtk_status);

        }

        if (msg_header == UNIHEADING_HEADER) {

            mrover::msg::RTKHeading heading;
            mrover::msg::RTKStatus rtk_status;
            mrover::msg::RTKFixType sol_status;

            if (tokens[ADRNAV_STATUS_POS] == "SOL_COMPUTED") {
                sol_status.fix_type = mrover::msg::RTKFixType::SOL_COMPUTED;
            }
            else if (tokens[ADRNAV_STATUS_POS] == "NO_CONVERGENCE") {
                sol_status.fix_type = mrover::msg::RTKFixType::NO_CONVERGENCE;
            }
            else if (tokens[ADRNAV_STATUS_POS] == "COV_TRACE") {
                sol_status.fix_type = mrover::msg::RTKFixType::COV_TRACE;
            }
            else {
                RCLCPP_WARN(get_logger(), "No RTK fix. Is the basestation on?");
                return;
            }

            rtk_status.sol_status = sol_status;

            float uniheading = stof(tokens[UNIHEADING_HEADING_POS]);

            heading.header = header;
            heading.heading = uniheading;

            rtk_status.header = header;

            heading_pub->publish(heading);
            heading_status_pub->publish(rtk_status);

        }

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

    // TODO: graceful keyboard interrupt
    
    return 0;

}