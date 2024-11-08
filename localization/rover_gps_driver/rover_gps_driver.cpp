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
        gps_status_pub = this->create_publisher<mrover::msg::FixStatus>("/gps_fix_status", 10);
        heading_pub = this->create_publisher<mrover::msg::Heading>("/heading/fix", 10);
        heading_status_pub = this->create_publisher<mrover::msg::FixStatus>("/heading_fix_status", 10);
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
            mrover::msg::FixStatus fix_status;
            mrover::msg::FixType fix_type;

            if (stoi(tokens[GNGGA_QUAL_POS]) == 0) {
                RCLCPP_WARN(get_logger(), "Invalid fix. Are we inside?");
                return;
            }
            else {
                RCLCPP_INFO(get_logger(), "Valid fix, %s satellites in use.", tokens[GNGGA_SATELLITES_POS].c_str());
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

            if (stoi(tokens[GNGGA_QUAL_POS]) == 5) {
                fix_type.fix = mrover::msg::FixType::FLOAT;
            }
            else if (stoi(tokens[GNGGA_QUAL_POS]) == 4) {
                fix_type.fix = mrover::msg::FixType::FIXED;
            }
            else {
                RCLCPP_WARN(get_logger(), "Position: No RTK fix. Has the basestation finished survey-in?");
                fix_type.fix = mrover::msg::FixType::NONE;
            }

            fix_status.fix_type = fix_type;
            fix_status.header = header;
            
            gps_status_pub->publish(fix_status);
            gps_pub->publish(nav_sat_fix);

        }

        // if (msg_header == RTKSTATUS_HEADER) {

        //     mrover::msg::RTKStatus fix_status;
        //     mrover::msg::FixType fix_type;

        //     if (tokens[RTKSTATUS_POS] == "NARROW_FLOAT") {
        //         fix_type.fix = mrover::msg::FixType::FLOAT;
        //     }
        //     else if (tokens[RTKSTATUS_POS] == "NARROW_INT") {
        //         fix_type.fix = mrover::msg::FixType::FIXED;
        //     }
        //     else {
        //         RCLCPP_WARN(get_logger(), "Position: No RTK fix. Has the basestation finished survey-in?");
        //         fix_type.fix = mrover::msg::FixType::NONE;
        //     }

        //     fix_status.fix_type = fix_type;
        //     fix_status.header = header;
            
        //     gps_status_pub->publish(fix_status);

        // }

        if (msg_header == UNIHEADING_HEADER) {

            mrover::msg::Heading heading;
            mrover::msg::FixStatus fix_status;
            mrover::msg::FixType fix_type;

            if (tokens[UNIHEADING_STATUS_POS] == "NARROW_FLOAT") {
                fix_type.fix = mrover::msg::FixType::FLOAT;
            }
            else if (tokens[UNIHEADING_STATUS_POS] == "NARROW_INT") {
                fix_type.fix = mrover::msg::FixType::FIXED;
            }
            else {
                RCLCPP_WARN(get_logger(), "Heading: no solution. Are both antennas plugged in?");
                return;
            }

            fix_status.fix_type = fix_type;

            float uniheading = stof(tokens[UNIHEADING_HEADING_POS]);

            heading.header = header;
            heading.heading = uniheading;

            fix_status.header = header;

            heading_pub->publish(heading);
            heading_status_pub->publish(fix_status);

        }

        if (msg_header == GPS_HEADER) {

            if (tokens[CNO_POS] == "") {
                RCLCPP_INFO(get_logger(), "GPS signal strength: 0");
            }
            else {
                RCLCPP_INFO(get_logger(), "GPS signal strength: %s", tokens[CNO_POS].c_str());
            }
        }

        if (msg_header == GLONASS_HEADER) {

            if (tokens[CNO_POS] == "") {
                RCLCPP_INFO(get_logger(), "GNONASS signal strength: 0");
            }
            else {
                RCLCPP_INFO(get_logger(), "GLONASS signal strength: %s", tokens[CNO_POS].c_str());
            }
        }

        if (msg_header == BEIDOU_HEADER) {

            if (tokens[CNO_POS] == "") {
                RCLCPP_INFO(get_logger(), "BeiDou signal strength: 0");
            }
            else {
                RCLCPP_INFO(get_logger(), "BeiDou signal strength: %s", tokens[CNO_POS].c_str());
            }
        }

        if (msg_header == GALILEO_HEADER) {

            if (tokens[CNO_POS] == "") {
                RCLCPP_INFO(get_logger(), "Galileo signal strength: 0");
            }
            else {
                RCLCPP_INFO(get_logger(), "Galileo signal strength: %s", tokens[CNO_POS].c_str());
            }
        }

        if (msg_header == QZSS_HEADER) {

            if (tokens[CNO_POS] == "") {
                RCLCPP_INFO(get_logger(), "QZSS signal strength: 0");
            }
            else {
                RCLCPP_INFO(get_logger(), "QZSS signal strength: %s", tokens[CNO_POS].c_str());
            }
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
