#include "can_device.hpp"
#include "messaging.hpp"
#include <memory>
#include <mrover/CAN.h>
#include <unordered_map>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <units.hpp>


namespace mrover {

    class ScienceBridge final : public rclcpp::Node {
    public:
        ScienceBridge() : Node{"science_bridge"} {
            tempPub = create_publisher<msg::TemperatureData>("science_temperature_data", 10);
            humidityPub = create_publisher<msg::HumidityData>("science_humidity_data", 10);
            oxygenPub = create_publisher<msg::OxygenDataData>("science_oxygen_data", 10);
            methanePub = create_publisher<msg::MethaneDataData>("science_methane_data", 10);
            uvPub = create_publisher<msg::UVData>("science_uv_data", 10);

            canSub = create_subscription<mrover::CAN>("can/science/in", 10, processCANData(mrover::CAN::ConstPtr const& msg));
        }

    private:
        void processCANData(mrover::CAN::ConstPtr const& msg) {

            // TODO - fix in future
            // ROS_ERROR("Source: %s Destination: %s", msg->source.c_str(), msg->destination.c_str());
            assert(msg->source == "science");
            assert(msg->destination == "jetson");



            // mrover::OutBoundScienceMessage const& message = *reinterpret_cast<mrover::OutBoundScienceMessage const*>(msg->data.data());

            // // This calls the correct process function based on the current value of the alternative
            // std::visit([&](auto const& messageAlternative) { processMessage(messageAlternative); }, message);
        }
    };

} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mrover::DifferentialDriveController>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
