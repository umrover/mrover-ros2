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
        rclclcpp::Service<srv::ServoSetPos>::SharedPtr setServoPositionService;
        // int serial_port; // unneeded?

        void process_rtcm(const rtcm_msgs::msg::Message::ConstSharedPtr &rtcm_msg); // this is not the move i think
        void process_unicore(std::string &unicore_msg);

        boost::asio::basic_serial_port<boost::asio::io_context::executor_type> serial;
        boost::asio::streambuf read_buffer;

        unsigned long baud;
        std::string port;
        std::string frame_id;


        auto setServoPositionServiceCallback(srv::ServoSetPos::Request::ConstSharedPtr const req, srv::ServoSetPos::Response::SharedPtr res) -> void {
            ServoSetPosition setPos;
            setPos.id = 0;
            setPos.isCounterClockwise = req->is_counterclockwise;
            setPos.radians = req->position;

            boost::asio::write(serial, boost::asio::buffer(reinterpret_cast<std::uint8_t*>(&setPos))); // MAKE THIS LINE (new boost serial stuff)

            //ssize_t bytes_written = write(serial_port, reinterpret_cast<std::uint8_t*>(&setPos), sizeof(setPos)); // ACT LIKE THIS LINE (old serialport stuff)
            res->success = true;
        }


    public:
        ArduinoBridge(boost::asio::io_context& io) : Node{"sa_arduino_hw_bridge"}, serial(io) {
            temperaturePub = create_publisher<sensor_msgs::msg::Temperature>("sa_temp_data", 10);
            humidityPub = create_publisher<sensor_msgs::msg::RelativeHumidity>("sa_humidity_data", 10);
            servoPositionPub = create_publisher<msg::Position>("sa_gear_diff_position", 10);

            setServoPositionService = create_service<srv::ServoSetPos>("sa_gear_diff_set_position", [this](srv::ServoSetPos::Request::ConstSharedPtr const req, srv::ServoSetPos::Response::SharedPtr res) {
                setServoPositionServiceCallback(req, res);
            });

            // NEW SERIAL

            declare_parameter("port_unicore", rclcpp::ParameterType::PARAMETER_STRING);
            declare_parameter("baud_unicore", rclcpp::ParameterType::PARAMETER_INTEGER);
            declare_parameter("frame_id", rclcpp::ParameterType::PARAMETER_STRING);

            port = get_parameter("port_unicore").as_string();
            baud = get_parameter("baud_unicore").as_int();
            frame_id = get_parameter("frame_id").as_string();

            serial.open(port);
            std::string port_string = std::to_string(baud);
            serial.set_option(boost::asio::serial_port_base::baud_rate(baud));



            // old serial!
            // char const* device = "/dev/ttyACM0"; // Replace with the appropriate device
            // int baudrate = B115200;              // Match the Arduino's baud rate

            // // Open the serial port
            // serial_port = open(device, O_RDWR | O_NOCTTY | O_SYNC);
            // if (serial_port < 0) {
            //     RCLCPP_FATAL_STREAM(get_logger(), "Unable to open arduino serial port");
            //     rclcpp::shutdown();
            // }

            // // Configure the serial port
            // termios tty{};
            // memset(&tty, 0, sizeof tty);
            // if (tcgetattr(serial_port, &tty) != 0) {
            //     close(serial_port);
            //     RCLCPP_FATAL_STREAM(get_logger(), "Unable to get serial port attributes");
            //     rclcpp::shutdown();
            // }

            // // Set baud rate
            // cfsetospeed(&tty, baudrate);
            // cfsetispeed(&tty, baudrate);

            // // 8N1 mode: 8 data bits, no parity, 1 stop bit
            // tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
            // tty.c_iflag &= ~IGNBRK;
            // tty.c_lflag = 0;     // No signaling chars, no echo, no canonical processing
            // tty.c_oflag = 0;     // No remapping, no delays
            // tty.c_cc[VMIN] = 1;  // Read blocks until at least 1 byte is available
            // tty.c_cc[VTIME] = 1; // Timeout for read

            // tty.c_iflag &= ~(IXON | IXOFF | IXANY);      // Disable software flow control
            // tty.c_cflag |= (CLOCAL | CREAD);             // Enable receiver, local mode
            // tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS); // No parity, 1 stop bit, no flow control

            // if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            //     close(serial_port);
            //     RCLCPP_FATAL_STREAM(get_logger(), "Unable to set serial port attributes");
            //     rclcpp::shutdown();
            // }
            // SERIAL END
        }

        void readSerialData(std::string serial_msg){
            //Read from serial_port and print out the HEADER_BYTE, message_id, Servo ID, present position
            ServoPositionData posData;
            TemperatureAndHumidityData tempHumidityData;

            std::vector<std::string> tokens;
            boost::split(tokens, serial_msg, boost::is_any_of(",;"));

            std::string msg_header = tokens[0];

            std_msgs::msg::Header header;
            //header.stamp = get_clock()->now(); // no? idt we need a stamp
            header.frame_id = frame_id;



            if(msg_header == HEADER_BYTE){
                if(tokens[1] == SERVO_POSITION_DATA){
                    posData.header = HEADER_BYTE;
                    posData.messageID = SERVO_POSITION_DATA;
                    posData.id = read_buffer[2];
                    
                    float radians;
                    memcpy(&radians, &read_buffer[3], sizeof(float));
                    ServoPositionData posData = {.radians = radians};
                }

                else if (tokens[1] == TEMPERATURE_HUMIDITY_DATA) {
                    tempHumidityData.header = HEADER_BYTE;
                    tempHumidityData.messageID = TEMPERATURE_HUMIDITY_DATA;
                    
                    float temperature;
                    memcpy(&temperature, &read_buffer[2], sizeof(float));
                    tempHumidityData.temperature = temperature;

                    float humidity;
                    memcpy(&humidity, &read_buffer[6], sizeof(float));
                    tempHumidityData.humidity = humidity;
                }
            }
            
            // split temp and humidity
            sensor_msgs::msg::Temperature temp;
            sensor_msgs::msg::RelativeHumidity humidity;
            float radians = posData.radians;

            temp.temperature = tempHumidityData.temperature;
            humidity.relative_humidity = tempHumidityData.humidity;

            //publish data
            temperaturePub->publish(temp);
            humidityPub->publish(humidity);

        }

        void ArduinoBridge::spin(){
            while (rclcpp::ok()) {
                boost::asio::read_until(serial, read_buffer, '\n');
                std::istream buffer(&read_buffer);
                std::string serial_msg;
                std::getline(buffer, serial_msg);
                readSerialData(serial_msg);

                rclcpp::spin_some(this->get_node_base_interface());
            }
        }

    }; // namespace mrover
} //namespace mrover



auto main(int argc, char** argv) -> int {

    rclcpp::init(argc, argv);

    boost::asio::io_context io;
    auto node = std::make_shared<mrover::ArduinoBridge>(io);

    node->spin();
    rclcpp::shutdown();
    
    return EXIT_SUCCESS;
}


///THE CODE CEMETERY
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