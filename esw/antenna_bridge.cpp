#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <units.hpp>
#include <units_eigen.hpp>

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>
#include <mrover/srv/adjust_motor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "motor_library/brushed.hpp"
#include "motor_library/brushless.hpp"

//listens for a throttle command
//send the throttle command to 1BM

namespace mrover {

    class AntennaBridge : public rclcpp::Node {

        using Controller = std::variant<BrushedController, BrushlessController<Meters>, BrushlessController<Revolutions>>;

    public:
        AntennaBridge() : rclcpp::Node{"antenna_bridge"} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {
            // mArmThrottleSub = create_subscription<msg::Throttle>("arm_throttle_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) { processThrottleCmd(msg); });
            mAntennaThrottleSub = create_subscription<msg::Throttle>("antenna_throttle_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) { processThrottleCmd(msg); });
            
        }

    private:
        // rclcpp::Subscription<msg::Throttle>::SharedPtr mArmThrottleSub;
        const std::string antenna_name = "antenna";
        std::shared_ptr<BrushedController> mAntennaMotor; 

        rclcpp::Subscription<msg::Throttle>::SharedPtr mAntennaThrottleSub;


        auto processThrottleCmd(msg::Throttle::ConstSharedPtr const& msg) -> void {
            if(msg->names.size() != msg->throttles.size()) {
                RCLCPP_ERROR(get_logger(), "Name count and value count mismatched");
                return;
            }

            for(std::size_t i = 0; i < msg->names.size(); ++i){
                std::string const& name = msg->name[i];

                if(name == antenna_name){
                    Dimensionless const& throttle = msg->throttles[i];
                    mAntenna->setDesiredThrottle(throttle);
                }

            }
        }
    
    };
} // namespace mrover

auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto antenna_bridge = std::make_shared<mrover::AntennaBridge>();
    antenna_bridge->init();
    rclcpp::spin(antenna_bridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
