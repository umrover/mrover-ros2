#include "rover_gps_driver.hpp"
#include <std_msgs/msg/detail/header__struct.hpp>
#include <string>

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


    std::string get_next_token(std::string& msg) {
        std::string delimeter = ",";
        std::string token = msg.substr(0,msg.find(delimeter));
        msg = msg.substr(msg.find(delimeter)+1);
        return token;
    }
    // just want to log the messages for now
    void RoverGPSDriver::process_unicore(std::string unicore_msg) {
        RCLCPP_INFO(get_logger(), ("Message: " + unicore_msg).c_str());
        std::string type = get_next_token(unicore_msg);
        
        std::string first_char = type.substr(0,1);
        std::string header = type.substr(1, 7);
        float lat;
        std::string lat_dir = "K"; // dummy value
        float lon;
        std::string lon_dir;
        float alt;


        // GP whatever
        if (first_char == "$") {
            std::string code = type.substr(1);
            if (code == "GNGGA") {
                // get lat/lon
                std::string utc = get_next_token(unicore_msg);
                lat = stof(get_next_token(unicore_msg));
                lat_dir = get_next_token(unicore_msg);
                lon = stof(get_next_token(unicore_msg));
                lon_dir = get_next_token(unicore_msg);
                for (int i = 0; i < 3; ++i) {
                    get_next_token(unicore_msg);
                }
                alt = stof(get_next_token(unicore_msg));
                
            } else {
                return;
            }
            

        } 
        else if (first_char == "#") {
            if (header == "ADRNAVA") {
                for (int i = 0; i < 10; i++) {
                    get_next_token(unicore_msg);
                }
                lat = stod(get_next_token(unicore_msg));
                lon = stod(get_next_token(unicore_msg));
                alt = stod(get_next_token(unicore_msg));
            }

        }

        sensor_msgs::msg::NavSatFix pose_data;
        pose_data.set__altitude(alt);
        if (lat_dir == "S") {
            lat = -lat;
        }
        pose_data.set__latitude(lat);
        if (lon_dir == "W") {
            lon = -lon;
        }
        pose_data.set__longitude(lon);
        
        // TODO: get real covariance
        std::array<double, 9> cov = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        
        // TODO fix this stuff and push
        pose_data.set__position_covariance(const &cov);

        auto header = std_msgs::msg::Header_();
        header.

        pose_data.set__header(const std_msgs::msg::Header_<allocator<void>> &_arg)

        
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
