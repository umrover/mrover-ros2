#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>

#include <mrover/msg/position.hpp>
#include <mrover/srv/servo_set_pos.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <messaging_sa_arduino.hpp>

//spa bridge takes info from serial and
//publishes to sa_temp_data and sa_humidity data
//and sa_gear_diff_position

//service/client from teleop to sa_gear_diff_set_position

//also make .srv

namespace mrover {
    using namespace messaging::arduino;

    class ArduinoBridge final : public rclcpp::Node {

    private:
        rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperaturePub;
        rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr humidityPub;
        rclcpp::Publisher<msg::Position>::SharedPtr servoPositionPub;
        rclcpp::Service<srv::ServoSetPos>::SharedPtr setServoPositionService;
        int serial_port;


        // auto readSerialData() -> void {
        //     //[TO-DO] read serial data in
        //     // Read from serial_port and print out the HEADER_BYTE, message_id, Servo ID, present position
        //     ServoPositionData posData;
        //     TemperatureAndHumidityData tempHumidityData;

        //     uint8_t read_buffer[2 + 8];
        //     ssize_t bytes_read = read(serial_port, &read_buffer, sizeof(read_buffer)); // temp/humidity double check the -1

        //     if (read_buffer[0] == HEADER_BYTE) {

        //         if (read_buffer[1] == SERVO_POSITION_DATA) {

        //             posData.header = HEADER_BYTE;
        //             posData.messageID = SERVO_POSITION_DATA;
        //             posData.id = read_buffer[2];

        //             float radians;
        //             memcpy(&radians, &read_buffer[3], sizeof(float));
        //             ServoPositionData posData = {
        //                     .radians = radians,
        //             };

        //         }

        //         else if (read_buffer[1] == TEMPERATURE_HUMIDITY_DATA) {

        //             tempHumidityData.header = HEADER_BYTE;
        //             tempHumidityData.messageID = TEMPERATURE_HUMIDITY_DATA;

        //             float temperature;
        //             memcpy(&temperature, &read_buffer[2], sizeof(float));
        //             tempHumidityData.temperature = temperature;

        //             float humidity;
        //             memcpy(&humidity, &read_buffer[6], sizeof(float));
        //             tempHumidityData.humidity = humidity;
        //         }
        //     }


        //     //verify header byte and message id
        //     if (posData.header == HEADER_BYTE && tempHumidityData.header == HEADER_BYTE && posData.message_id == MSG_SERVO_POSITION_DATA && tempHumidityData.message_id == MSG_TEMPERATURE_HUMIDITY) {
        //         //grab data
        //         sensor_msgs::msg::Temperature temp;
        //         sensor_msgs::msg::RelativeHumidity humidity;
        //         float radians = posData.radians;

        //         temp.temperature = tempHumidityData.temperature;
        //         humidity.relative_humidity = tempHumidityData.humidity;


        //         //publish data
        //         temperaturePub->publish(temp);
        //         humidityPub->publish(humidity);
        //         // servoPositionPub->publish(radians);
        //     }

        //     //else do nothing
        // }

        auto setServoPositionServiceCallback(srv::ServoSetPos::Request::ConstSharedPtr const req, srv::ServoSetPos::Response::SharedPtr res) -> void {
            ServoSetPosition setPos;
            setPos.id = 0;
            setPos.isCounterClockwise = req->is_counterclockwise;
            setPos.radians = req->position;

            ssize_t bytes_written = write(serial_port, reinterpret_cast<std::uint8_t*>(&setPos), sizeof(setPos));

            res->success = true;
        }


    public:
        ArduinoBridge() : Node{"sa_arduino_hw_bridge"} {
            temperaturePub = create_publisher<sensor_msgs::msg::Temperature>("sa_temp_data", 10);
            humidityPub = create_publisher<sensor_msgs::msg::RelativeHumidity>("sa_humidity_data", 10);
            servoPositionPub = create_publisher<msg::Position>("sa_gear_diff_position", 10);

            setServoPositionService = create_service<srv::ServoSetPos>("sa_gear_diff_set_position", [this](srv::ServoSetPos::Request::ConstSharedPtr const req, srv::ServoSetPos::Response::SharedPtr res) {
                setServoPositionServiceCallback(req, res);
            });

            char const* device = "/dev/ttyACM0"; // Replace with the appropriate device
            int baudrate = B115200;              // Match the Arduino's baud rate

            // Open the serial port
            serial_port = open(device, O_RDWR | O_NOCTTY | O_SYNC);
            if (serial_port < 0) {
                RCLCPP_FATAL_STREAM(get_logger(), "Unable to open arduino serial port");
                rclcpp::shutdown();
            }

            // Configure the serial port
            termios tty{};
            memset(&tty, 0, sizeof tty);
            if (tcgetattr(serial_port, &tty) != 0) {
                close(serial_port);
                RCLCPP_FATAL_STREAM(get_logger(), "Unable to get serial port attributes");
                rclcpp::shutdown();
            }

            // Set baud rate
            cfsetospeed(&tty, baudrate);
            cfsetispeed(&tty, baudrate);

            // 8N1 mode: 8 data bits, no parity, 1 stop bit
            tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
            tty.c_iflag &= ~IGNBRK;
            tty.c_lflag = 0;     // No signaling chars, no echo, no canonical processing
            tty.c_oflag = 0;     // No remapping, no delays
            tty.c_cc[VMIN] = 1;  // Read blocks until at least 1 byte is available
            tty.c_cc[VTIME] = 1; // Timeout for read

            tty.c_iflag &= ~(IXON | IXOFF | IXANY);      // Disable software flow control
            tty.c_cflag |= (CLOCAL | CREAD);             // Enable receiver, local mode
            tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS); // No parity, 1 stop bit, no flow control

            if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
                close(serial_port);
                RCLCPP_FATAL_STREAM(get_logger(), "Unable to set serial port attributes");
                rclcpp::shutdown();
            }
        }
    }; // namespace mrover
} //namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::ArduinoBridge>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
