#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <units.hpp>

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>
#include <mrover/srv/adjust_motor.hpp>

#include "motor_library/brushed.hpp"

namespace mrover {

    class AntennaBridge : public rclcpp::Node {

    public:
        AntennaBridge() : rclcpp::Node{"antenna_bridge"} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {
            mController = std::make_shared<BrushedController>(shared_from_this(), "rpi", antennaName);

            mThrottleSub = create_subscription<msg::Throttle>("antenna_throttle_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) { processThrottleCmd(msg); });
        }

    private:
        std::string const antennaName = "antenna";
        std::shared_ptr<BrushedController> mController;

        rclcpp::Subscription<msg::Throttle>::SharedPtr mThrottleSub;

        auto processThrottleCmd(msg::Throttle::ConstSharedPtr const& msg) -> void {
            if (msg->names.size() != 1 || msg->throttles.size() != 1) {
                RCLCPP_ERROR(get_logger(), "Both name and throttle should be of size 1 (just antenna)!");
                return;
            }

            std::string const& name = msg->names[0];
            if (name == antennaName) {
                Dimensionless const& throttle = msg->throttles[0];
                mController->setDesiredThrottle(throttle);
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
