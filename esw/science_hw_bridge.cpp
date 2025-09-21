#include <cstring>
#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/float64__struct.hpp>
#include <std_msgs/msg/float64.hpp>

#include <can_device.hpp>
#include <messaging_science.hpp>
#include <units.hpp>


namespace mrover {

    class ScienceBridge final : public rclcpp::Node {

    private:
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr geigerPub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr hydrogenPub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ozonePub;

        rclcpp::Subscription<msg::CAN>::ConstSharedPtr canSub;

        void processMessage(mrover::SensorData const& message) {
            switch (static_cast<ScienceDataID>(message.id)) {
                case ScienceDataID::GEIGER: {
                    std_msgs::msg::Float64 msg;
                    msg.data = message.data;
                    geigerPub->publish(msg);
                    break;
                }

                case ScienceDataID::HYDROGEN: {
                    std_msgs::msg::Float64 msg;
                    msg.data = message.data;
                    hydrogenPub->publish(msg);
                    break;
                }

                case ScienceDataID::OZONE: {
                    std_msgs::msg::Float64 msg;
                    msg.data = message.data;
                    ozonePub->publish(msg);
                    break;
                }
            }
        }

        void processCANData(msg::CAN::ConstSharedPtr const& msg) {
            SensorData const& message = *reinterpret_cast<SensorData const*>(msg->data.data());
            processMessage(message);
        }

    public:
        ScienceBridge() : Node {"science_hw_bridge"} {}

        void init() {
            geigerPub = create_publisher<std_msgs::msg::Float64>("science_geiger_data", 10);
            hydrogenPub = create_publisher<std_msgs::msg::Float64>("science_hydrogen_data", 10);
            ozonePub = create_publisher<std_msgs::msg::Float64>("science_ozone_data", 10);

            canSub = create_subscription<msg::CAN>("/can/science/in", 10, [this](msg::CAN::ConstSharedPtr const& msg) { processCANData(msg); });
        }
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto scienceBridge = std::make_shared<mrover::ScienceBridge>();
    scienceBridge->init();
    rclcpp::spin(scienceBridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
