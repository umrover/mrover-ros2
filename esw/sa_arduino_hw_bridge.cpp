#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <messaging_sa_arduino.hpp>

//spa bridge takes info from serial and 
//publishes to sa_temp_data and sa_humidity data
//and sa_gear_diff_position


//service/client from teleop to sa_gear_diff_set_position

//also make .srv 

namespace mrover{
    class ArduinoBridge final : public rclcpp::Node {
        private:
            rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr tempPub;
            rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr humidityPub;
            rclcpp::Publisher<std_msgs::Float32>::SharedPtr readServoPos;
            uint8_t HEADER_BYTE = 0xA5;
            uint8_t MSG_SERVO_SET_POSITION = 0x00;
            uint8_t MSG_SERVO_POSITION_DATA = 0x01;
            uint8_t MSG_TEMPERATURE_HUMIDITY = 0x02;
            int serial_port;

                        

            void readSerialData(){

                //[TO-DO] read serial data in 
                // Read from serial_port and print out the HEADER_BYTE, message_id, Servo ID, present position
                ServoPositionData posData;
                TemperatureandHumidityData tempHumidityData;

                uint8_t read_buffer[2 + 8];
                ssize_t bytes_read = read(serial_port, &read_buffer, sizeof(read_buffer)); // temp/humidity double check the -1

                if (read_buffer[0] == HEADER_BYTE){

                    if (read_buffer[1] == MSG_SERVO_POSITION_DATA) {

                        posData.header = HEADER_BYTE;
                        posData.message_id = MSG_SERVO_POSITION_DATA;
                        posData.id = read_buffer[2];

                        float radians;
                        memcpy(&radians, &read_buffer[3], sizeof(float));
                        ServoPositionData posData.radians = radians;

                    }

                    else if (read_buffer[1] == MSG_TEMPERATURE_HUMIDITY) {

                        tempHumidityData.header = HEADER_BYTE;
                        tempHumidityData.message_id = MSG_TEMPERATURE_HUMIDITY;

                        float temperature;
                        memcpy(&temperature, &read_buffer[2], sizeof(float));
                        tempHumidityData.temperature = temperature;

                        float humidity;
                        memcpy(&humidity, &read_buffer[6], sizeof(float));
                        tempHumidityData.humidity = humidity;

                    }

                } 
                

                //verify header byte and message id
                if (posData.header == HEADER_BYTE && tempHumidityData.header == HEADER_BYTE
                   && posData.message_id == MSG_SERVO_POSITION_DATA 
                   && tempHumidityData.message_id == MSG_TEMPERATURE_HUMIDITY) {
                        //grab data
                        sensor_msgs::msg::Temperature temp;
                        sensor_msgs::msg::RelativeHumidity humidity;
                        float32 radians = posData.radians;

                        temp.temperature = tempHumidityData.temperature;
                        humidity.relative_humidity = tempHumidityData.humidity;


                        //publish data
                        tempPub->publish(temp);
                        humidityPub->publish(humidity);
                        readServoPos->publish(radians);
                   }

                //else do nothing
            }

            void setServoPosition(srv::SetServoPos::Request::ConstSharedPtr const& req, srv::SetServoPos::Response::SharedPtr &resp){

                //format servo position request into ServoSetPosition struct
                ServoSetPosition setPos;
                setPos.id = req->id;
                setPos.is_counterclockwise = req->is_counterclockwise;
                setPos.radians = req->radians;

                //[TO-DO] send servo position over serial 
                // Message size: 1 byte for HEADER + 1 byte for message_id + 1 byte for ID and flags + sizeof(float) for radians

                uint8_t message[1 + 1 + 1 + sizeof(float)];

                // Set the HEADER_BYTE and message_id
                message[0] = HEADER_BYTE;
                message[1] = MSG_SERVO_SET_POSITION;  // Set message_id

                // Pack the ID and is_counterclockwise flag into the third byte
                uint8_t id_flags = (setPos.id & 0x0F) << 1 | (setPos.is_counterclockwise & 0x01);
                message[2] = id_flags;  // Set ID and direction (CW/CCW)

                // Pack radians as float (4 bytes)
                memcpy(&message[3], &setPos.radians, sizeof(float));

                // Send the message to the serial port
                ssize_t bytes_written = write(serial_port, message, sizeof(message));

                //set response to success
                resp->success = true;
            }


        
        public:
            ArduinoBridge() : Node{"sa_arduino_hw_bridge"}{
                tempPub = create_publisher<sensor_msgs::msg::Temperature>("sa_temp_data", 10);
                humidityPub = create_publisher<sensor_msgs::msg::RelativeHumidity>("sa_humidity_data",10);
                readServoPos = create_publisher<std_msgs::Float32>("sa_gear_diff_position", 10);
                
                setServoPos = create_service<srv::ServoSetPos>("",[this](srv::SetServoPos::Request::ConstSharedPtr const& req, srv::SetServoPos::Response::SharedPtr &resp){
                    setServoPos(req, resp);
                });

                const char *device = "/dev/ttyACM0"; // Replace with the appropriate device
                int baudrate = B115200;               // Match the Arduino's baud rate

                // Open the serial port
                serial_port = open(device, O_RDWR | O_NOCTTY | O_SYNC);
                if (serial_port < 0) {
                    std::cerr << "Error: Unable to open serial port " << device << std::endl;
                    return 1;
                }

                // Configure the serial port
                struct termios tty;
                memset(&tty, 0, sizeof tty);
                if (tcgetattr(serial_port, &tty) != 0) {
                    std::cerr << "Error: Unable to get terminal attributes" << std::endl;
                    close(serial_port);
                    return 1;
                }

                // Set baud rate
                cfsetospeed(&tty, baudrate);
                cfsetispeed(&tty, baudrate);

                // 8N1 mode: 8 data bits, no parity, 1 stop bit
                tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
                tty.c_iflag &= ~IGNBRK;
                tty.c_lflag = 0;             // No signaling chars, no echo, no canonical processing
                tty.c_oflag = 0;             // No remapping, no delays
                tty.c_cc[VMIN] = 1;          // Read blocks until at least 1 byte is available
                tty.c_cc[VTIME] = 1;         // Timeout for read

                tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control
                tty.c_cflag |= (CLOCAL | CREAD);         // Enable receiver, local mode
                tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS); // No parity, 1 stop bit, no flow control

                if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
                    std::cerr << "Error: Unable to set terminal attributes" << std::endl;
                    close(serial_port);
                    return 1;
                }

            }
            
    }
} //namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::ArduinoBridge>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
