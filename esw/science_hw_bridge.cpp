#include <cstring>
#include <memory>

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <std_msgs/msg/float64.hpp>

#include <can_device.hpp>
#include <messaging_science.hpp>
#include <units.hpp>

static constexpr int SERIAL_INPUT_MSG_SIZE = 64;

namespace mrover {

    class ScienceBridge final : public rclcpp::Node {

    private:
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr geigerPub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr hydrogenPub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ozonePub;

        boost::asio::serial_port serial;
        std::string serial_port;
        boost::asio::io_service& io;
        std::vector<unsigned char> buffer;
        unsigned long serial_baud_rate{};

        // rclcpp::Subscription<msg::CAN>::ConstSharedPtr canSub;

        // void processMessage(mrover::SensorData const& message) {
        //     switch (static_cast<ScienceDataID>(message.id)) {
        //         case ScienceDataID::GEIGER: {
        //             std_msgs::msg::Float64 msg;
        //             msg.data = message.data;
        //             geigerPub->publish(msg);
        //             break;
        //         }

        //         case ScienceDataID::HYDROGEN: {
        //             std_msgs::msg::Float64 msg;
        //             msg.data = message.data;
        //             hydrogenPub->publish(msg);
        //             break;
        //         }

        //         case ScienceDataID::OZONE: {
        //             std_msgs::msg::Float64 msg;
        //             msg.data = message.data;
        //             ozonePub->publish(msg);
        //             break;
        //         }
        //     }
        // }

        // void processCANData(msg::CAN::ConstSharedPtr const& msg) {
        //     SensorData const& message = *reinterpret_cast<SensorData const*>(msg->data.data());
        //     processMessage(message);
        // }

        auto startAsyncReadThread() -> void {
            std::thread([this]() {
                asyncReadSerial();
                io.run();
            }).detach();
        }

        auto asyncReadSerial() -> void {
            boost::asio::async_read_until(serial, boost::asio::dynamic_buffer(buffer), '\n',
                [this](boost::system::error_code const& ec, std::size_t len) {
                    if (ec) {
                        RCLCPP_ERROR(this->get_logger(), "Serial read failed: %s", ec.message().c_str());
                    }
                    std::string line(buffer.begin(), buffer.begin() + static_cast<long>(len));
                    buffer.erase(buffer.begin(), buffer.begin() + static_cast<long>(len));
                    parseAndPublishBuffer(line);

                    asyncReadSerial();
                });
        }

        auto parseAndPublishBuffer(std::string& line) const -> void {
            // read from serial port and parse data
            std::istringstream ss(line);
            std::string token;

            float hydrogen_ppm = 0;
            float ozone_ppb = 0;
            float geiger_count = 0;

            if (std::getline(ss, token, ',')) {
                hydrogen_ppm = std::stof(token);
            }
            if (std::getline(ss, token, ',')) {
                ozone_ppb = std::stof(token);
            }
            if (std::getline(ss, token)) {
                geiger_count = std::stof(token);
            }

            // Publish to ROS
            std_msgs::msg::Float64 msg;
            msg.data = hydrogen_ppm;
            hydrogenPub->publish(msg);

            msg.data = ozone_ppb;
            ozonePub->publish(msg);

            msg.data = geiger_count;
            geigerPub->publish(msg);
        }

    public:
        explicit ScienceBridge(boost::asio::io_service& io_in) : rclcpp::Node{"science_hw_bridge"}, serial(io_in), io(io_in), buffer(SERIAL_INPUT_MSG_SIZE) {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        void init() {
            geigerPub = create_publisher<std_msgs::msg::Float64>("science_geiger_data", 10);
            hydrogenPub = create_publisher<std_msgs::msg::Float64>("science_hydrogen_data", 10);
            ozonePub = create_publisher<std_msgs::msg::Float64>("science_ozone_data", 10);

            // canSub = create_subscription<msg::CAN>("/can/science/in", 10, [this](msg::CAN::ConstSharedPtr const& msg) { processCANData(msg); });
        
            serial_port = this->declare_parameter<std::string>("port_unicore", "/dev/science");
            serial_baud_rate = this->declare_parameter<int>("baud_unicore", 115200);

            boost::system::error_code ec;
            serial.open(serial_port, ec);
            if (ec) {
                RCLCPP_FATAL(this->get_logger(), "Couldn't open serial port: %s", ec.message().c_str());
            } else {
                serial.set_option(boost::asio::serial_port_base::baud_rate(serial_baud_rate));
                // begin serial reads on a separate thread
                startAsyncReadThread();
            }
        }

        auto stop() -> void {
            boost::system::error_code ec;
            serial.close(ec);
            if (ec) {
                RCLCPP_WARN(this->get_logger(), "Failed to close serial port: %s", ec.message().c_str());
            }
        }
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    boost::asio::io_service io;
    auto scienceBridge = std::make_shared<mrover::ScienceBridge>(io);
    scienceBridge->init();
    rclcpp::spin(scienceBridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
