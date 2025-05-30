#include "rover_gps_driver.hpp"

namespace mrover {

    RoverGPSDriver::RoverGPSDriver(boost::asio::io_context& io) : Node("rover_gps_driver"), serial(io) {
        
        // connect to serial
        declare_parameter("port_unicore", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("baud_unicore", rclcpp::ParameterType::PARAMETER_INTEGER);
        declare_parameter("frame_id", rclcpp::ParameterType::PARAMETER_STRING);
        declare_parameter("ref_lat", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("ref_lon", rclcpp::ParameterType::PARAMETER_DOUBLE);
        declare_parameter("ref_alt", rclcpp::ParameterType::PARAMETER_DOUBLE);

        port = get_parameter("port_unicore").as_string();
        baud = get_parameter("baud_unicore").as_int();
        frame_id = get_parameter("frame_id").as_string();

        serial.open(port);
        std::string port_string = std::to_string(baud);
        serial.set_option(boost::asio::serial_port_base::baud_rate(baud));

        RCLCPP_INFO(get_logger(), "Connected to GPS via serial!");
        
        // publishers and subscribers
        gps_pub = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
        gps_status_pub = this->create_publisher<mrover::msg::FixStatus>("/gps_fix_status", 10);
        // heading_pub = this->create_publisher<mrover::msg::Heading>("/heading/fix", 10);
        // heading_status_pub = this->create_publisher<mrover::msg::FixStatus>("/heading_fix_status", 10);
        // velocity_pub = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/velocity/fix", 10);
        // velocity_status_pub = this->create_publisher<mrover::msg::FixStatus>("/velocity_fix_status", 10);
        satellite_signal_pub = this->create_publisher<mrover::msg::SatelliteSignal>("/gps/satellite_signal", 10);
        rtcm_sub = this->create_subscription<rtcm_msgs::msg::Message>("/rtcm", 10, [&](rtcm_msgs::msg::Message::ConstSharedPtr const& rtcm_message) {
            process_rtcm(rtcm_message);
        });

        satellite_signals.insert({"GPS", std::vector<uint8_t>()});
        satellite_signals.insert({"GLONASS", std::vector<uint8_t>()});
        satellite_signals.insert({"BeiDou", std::vector<uint8_t>()});
        satellite_signals.insert({"Galileo", std::vector<uint8_t>()});
        satellite_signals.insert({"QZSS", std::vector<uint8_t>()});

        headers_to_constellations.insert({"$GPGSV", "GPS"});
        headers_to_constellations.insert({"$GLGSV", "GLONASS"});
        headers_to_constellations.insert({"$GBGSV", "BeiDou"});
        headers_to_constellations.insert({"$GAGSV", "Galileo"});
        headers_to_constellations.insert({"$GQGSV", "QZSS"});
    }

    void RoverGPSDriver::process_rtcm(const rtcm_msgs::msg::Message::ConstSharedPtr &rtcm_msg) {
        boost::asio::write(serial, boost::asio::buffer(rtcm_msg->message));
    }

    void RoverGPSDriver::process_unicore(std::string &unicore_msg) {

        std::vector<std::string> tokens;
        boost::split(tokens, unicore_msg, boost::is_any_of(",;*"));

        std::string msg_header = tokens[0];

        std_msgs::msg::Header header;
        header.stamp = get_clock()->now();
        header.frame_id = frame_id;

        try {

            if (msg_header == "$GNGGA") {

                if (tokens.size() != 16) { return; }

                sensor_msgs::msg::NavSatFix nav_sat_fix;
                mrover::msg::FixStatus fix_status;
                mrover::msg::FixType fix_type;
                nav_sat_fix.header = header;
                fix_status.header = header;

                if (stoi(tokens[6]) == 0) {
                    RCLCPP_WARN(get_logger(), "Invalid fix. Are we inside?");

                    fix_type.fix = mrover::msg::FixType::NO_SOL;
                    fix_status.fix_type = fix_type;

                    nav_sat_fix.latitude = get_parameter("ref_lat").as_double();
                    nav_sat_fix.longitude = get_parameter("ref_lon").as_double();
                    nav_sat_fix.altitude = get_parameter("ref_alt").as_double();

                    gps_pub->publish(nav_sat_fix);
                    gps_status_pub->publish(fix_status);
                    return;
                }
                else {
                    RCLCPP_INFO(get_logger(), "Valid fix, %s satellites in use.", tokens[7].c_str());
                }
                
                uint16_t lat_deg = stoi(tokens[2].substr(0, 2));
                double lat_min = stod(tokens[2].substr(2, 13));
                uint16_t lon_deg = stoi(tokens[4].substr(0, 3));
                double lon_min = stod(tokens[4].substr(3, 14));
                double alt = stod(tokens[9]);

                // 60 minutes = 1 degree
                double lat = lat_deg + lat_min / 60;
                double lon = lon_deg + lon_min / 60;

                char lat_dir = tokens[3][0];
                char lon_dir = tokens[5][0];

                if (lat_dir == 'S') {
                    lat = -lat;
                }
                if (lon_dir == 'W') {
                    lon = -lon;
                }

                nav_sat_fix.latitude = lat;
                nav_sat_fix.longitude = lon;
                nav_sat_fix.altitude = alt;

                if (stoi(tokens[6]) == 5) {
                    fix_type.fix = mrover::msg::FixType::FLOAT;
                }
                else if (stoi(tokens[6]) == 4) {
                    fix_type.fix = mrover::msg::FixType::FIXED;
                }
                else {
                    RCLCPP_WARN(get_logger(), "Position: No RTK fix. Has the basestation finished survey-in?");
                    fix_type.fix = mrover::msg::FixType::NONE;
                }

                fix_status.fix_type = fix_type;
                
                gps_pub->publish(nav_sat_fix);
                gps_status_pub->publish(fix_status);

            }

            // if (msg_header == "#UNIHEADINGA") {

            //     if (tokens.size() != 28) { return; }

            //     mrover::msg::Heading heading;
            //     mrover::msg::FixStatus fix_status;
            //     mrover::msg::FixType fix_type;
            //     heading.header = header;
            //     fix_status.header = header;

            //     if (tokens[11] == "NARROW_FLOAT") {
            //         fix_type.fix = mrover::msg::FixType::FLOAT;
            //     }
            //     else if (tokens[11] == "NARROW_INT") {
            //         fix_type.fix = mrover::msg::FixType::FIXED;
            //     }
            //     else {
            //         RCLCPP_WARN(get_logger(), "Heading: no solution. Are both antennas plugged in?");

            //         fix_type.fix = mrover::msg::FixType::NO_SOL;
            //         fix_status.fix_type = fix_type;
            //         heading.heading = 0;

            //         heading_pub->publish(heading);
            //         heading_status_pub->publish(fix_status);
            //         return;
            //     }

            //     fix_status.fix_type = fix_type;

            //     float uniheading = stof(tokens[13]);
            //     heading.heading = uniheading;

            //     heading_pub->publish(heading);
            //     heading_status_pub->publish(fix_status);

            // }

            // if (msg_header == "#BESTNAVA") {

            //     if (tokens.size() != 41) { return; }

            //     geometry_msgs::msg::Vector3Stamped velocity;
            //     mrover::msg::FixStatus fix_status;
            //     mrover::msg::FixType fix_type;
            //     velocity.header = header;
            //     fix_status.header = header;

            //     if (tokens[32] == "DOPPLER_VELOCITY") {
            //         fix_type.fix = mrover::msg::FixType::NONE;
            //     }
            //     else {
            //         RCLCPP_WARN(get_logger(), "Velocity: no solution. Are we inside?");

            //         fix_type.fix = mrover::msg::FixType::NO_SOL;
            //         fix_status.fix_type = fix_type;

            //         velocity.vector.x = 0;
            //         velocity.vector.y = 0;
            //         velocity.vector.z = 0;

            //         velocity_pub->publish(velocity);
            //         velocity_status_pub->publish(fix_status);
            //         return;
            //     }

            //     fix_status.fix_type = fix_type;

            //     float gps_velocity = stof(tokens[35]);
            //     float gps_dir = stof(tokens[36]);

            //     velocity.vector.x = std::sin(gps_dir) * gps_velocity;
            //     velocity.vector.y = std::cos(gps_dir) * gps_velocity;
            //     velocity.vector.z = 0;

            //     velocity_pub->publish(velocity);
            //     velocity_status_pub->publish(fix_status);

            // }

            if (msg_header == "$GPGSV" || msg_header == "$GLGSV" || msg_header == "$GBGSV" || msg_header == "$GAGSV" || msg_header == "$GQGSV") {

                if (tokens.size() < 10 || (tokens.size() - 10) % 4 != 0 || tokens.size() > 26) { return; }
                if (stoi(tokens[3]) == 0) { return; }

                mrover::msg::SatelliteSignal signal;
                signal.constellation = headers_to_constellations[msg_header];
                for (size_t i = 7; i < tokens.size(); i += 4) {
                    satellite_signals[signal.constellation].push_back(stoi(tokens[i]));
                }
                if (stoi(tokens[1]) == stoi(tokens[2]) || satellite_signals[signal.constellation].size() > MAX_SATELLITE_SIGNAL_SIZE) {
                    signal.signal_strengths = satellite_signals[signal.constellation];
                    satellite_signal_pub->publish(signal);
                    satellite_signals[signal.constellation].clear();
                }
                
            }

        }
        catch(const std::invalid_argument& e) {
            RCLCPP_WARN(get_logger(), "Invalid argument: %s", e.what());
            return;
        }
        catch(...) {
            RCLCPP_WARN(get_logger(), "Exception caught.");
            return;
        }
        
    }


    void RoverGPSDriver::spin() {
        while (rclcpp::ok()) {
            try{
                boost::asio::read_until(serial, read_buffer, '\n');
                std::istream buffer(&read_buffer);
                std::string unicore_msg;
                std::getline(buffer, unicore_msg);
                process_unicore(unicore_msg);
            }catch(...){
                // Do nothing
                RCLCPP_WARN(get_logger(), "Exception caught while reading from the serial port.");
            }
            
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

    void RoverGPSDriver::stop() {
        boost::system::error_code e;
        serial.close(e);
        if (e) {
            RCLCPP_WARN(this->get_logger(), "Failed to close serial port: %s", e.message().c_str());
        }
    }

}

int main(int argc, char**argv) {

    rclcpp::init(argc, argv);

    boost::asio::io_context io;
    auto node = std::make_shared<mrover::RoverGPSDriver>(io);

    node->spin();

    node->stop();
    rclcpp::shutdown();
    
    return 0;

}
