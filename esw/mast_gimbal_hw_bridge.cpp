#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <units.hpp>

#include <mrover/msg/controller_state.hpp>
#include <mrover/srv/servo_position.hpp>

#include <servo.hpp>
#include <u2d2.hpp>

namespace mrover {

    class MastGimbalHWBridge : public rclcpp::Node {

    public:
        MastGimbalHWBridge() : Node{"mast_gimbal_hw_bridge"} {
            // all initialization is done in the init() function to allow for the usage of shared_from_this()
        }

        auto init() -> void {
            // parse parameters
            std::vector<ParameterWrapper> parameters = {
            };
            ParameterWrapper::declareParameters(this, parameters);

            for (std::string const& servoName: mServoNames) {
                auto servo = std::make_shared<Servo>(shared_from_this(), servoName);
                mServos.insert_or_assign(servoName, servo);
                mControllerState.names.push_back(servoName);
            }

            mTimerGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            mServiceGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            auto subOptions = rclcpp::SubscriptionOptions();
            subOptions.callback_group = mServiceGroup;

            mPositionService = this->create_service<srv::ServoPosition>(
                    "gimbal_servo",
                    [this](srv::ServoPosition::Request::SharedPtr const& req, srv::ServoPosition::Response::SharedPtr const& res) -> void {
                        servoPositionCallback(req, res);
                    },
                    rmw_qos_profile_services_default,
                    mServiceGroup);

            mPublishTimer = this->create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() -> void { publishDataCallback(); },
                    mTimerGroup);

            mGimbalStatePub = this->create_publisher<msg::ControllerState>("gimbal_controller_state", 10);
        }

    private:
        std::vector<std::string> mServoNames = {"gimbal_pitch", "gimbal_yaw"};
        std::unordered_map<std::string, std::shared_ptr<Servo>> mServos;

        rclcpp::CallbackGroup::SharedPtr mServiceGroup;
        rclcpp::CallbackGroup::SharedPtr mTimerGroup;

        rclcpp::Service<srv::ServoPosition>::SharedPtr mPositionService;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr mGimbalStatePub;
        rclcpp::TimerBase::SharedPtr mPublishTimer;
        msg::ControllerState mControllerState;

        auto servoPositionCallback(srv::ServoPosition::Request::SharedPtr const& req, srv::ServoPosition::Response::SharedPtr const& res) -> void {
            size_t const n = req->names.size();
            res->at_tgts.resize(n, false);

            for (size_t i = 0; i < n; ++i) {
                if (auto const it = mServos.find(req->names[i]); it != mServos.end()) {
                    it->second->setGoalPosition(Radians{req->positions[i]});
                    res->at_tgts[i] = true;
                }
            }
        }

        auto publishDataCallback() -> void {
            mControllerState.names.clear();
            mControllerState.positions.clear();
            mControllerState.velocities.clear();
            mControllerState.currents.clear();
            mControllerState.errors.clear();
            mControllerState.states.clear();
            mControllerState.limits_hit.clear();

            mControllerState.header.stamp = now();

            for (auto const& [name, servo]: mServos) {
                mControllerState.names.push_back(name);
                mControllerState.positions.push_back(static_cast<float>(servo->getPosition().get()));
                mControllerState.velocities.push_back(std::numeric_limits<float>::quiet_NaN());
                mControllerState.currents.push_back(std::numeric_limits<float>::quiet_NaN());
                mControllerState.errors.emplace_back("");
                mControllerState.states.emplace_back("");
                mControllerState.limits_hit.push_back(0);
            }

            mGimbalStatePub->publish(mControllerState);
        }
    };
} // namespace mrover


auto main(int const argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto const mast_gimbal_hw_bridge = std::make_shared<mrover::MastGimbalHWBridge>();
    mast_gimbal_hw_bridge->init();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(mast_gimbal_hw_bridge);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
