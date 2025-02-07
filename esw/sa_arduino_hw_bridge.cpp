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

            void readSerialData(){
                //[TO-DO] read serial data in 
                ServoPositionData posData;
                TemperatureandHumidityData tempHumidityData;

                //verify header byte and message id
                if(posData.header == HEADER_BYTE && tempHumidityData.header == HEADER_BYTE
                   && posData.message_id == 0x00 && tempHumidityData.message_id == 0x02){
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
                setPos.is_counterclockwise = req->is_counterclockwise;
                setPos.radians = req->position;

                //[TO-DO] send servo position over serial 

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
            }
            
    }
} //namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::ArduinoBridge>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
