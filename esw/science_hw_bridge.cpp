#include <cstddef>
#include <cstring>
#include <limits>
#include <memory>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/srv/servo_position.hpp>

#include <brushed.hpp>
#include <science.hpp>
#include <servo.hpp>
#include <u2d2.hpp>


namespace mrover {
    class ScienceHWBridge : public rclcpp::Node {

        using Controller = std::variant<
                std::shared_ptr<BrushedController<Meters>>,
                std::shared_ptr<BrushedController<Radians>>>;

    public:
        ScienceHWBridge() : Node{"science_hw_bridge"} {}

        auto init() -> void {
            // parse parameters
            std::vector<ParameterWrapper> parameters = {
                    {"brush_step", mBrushStep.rep, 0.25},
            };
            ParameterWrapper::declareParameters(this, parameters);

            mScienceBoard = std::make_shared<ScienceBoard>(shared_from_this(), "jetson", "science");
            mAuger = std::make_shared<BrushedController<Radians>>(shared_from_this(), "jetson", "auger");
            mLinearActuator = std::make_shared<BrushedController<Meters>>(shared_from_this(), "jetson", "linear_actuator");
            mFunnelServo = std::make_shared<Servo>(shared_from_this(), "funnel");
            mBrushServo = std::make_shared<Servo>(shared_from_this(), "brush");

            mServiceGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            auto subOptions = rclcpp::SubscriptionOptions();
            subOptions.callback_group = mServiceGroup;

            mFunnelPositionService = this->create_service<srv::ServoPosition>(
                    "sp_funnel_servo",
                    [this](srv::ServoPosition::Request::SharedPtr const& req, srv::ServoPosition::Response::SharedPtr const& res) -> void {
                        servoPositionCallback(req, res);
                    },
                    rmw_qos_profile_services_default,
                    mServiceGroup);

            mSPThrottleSub = create_subscription<msg::Throttle>("sp_thr_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) -> void { processThrottleCmd(msg); });

            mPublishDataTimer = create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() -> void { publishDataCallback(); });
            mControllerStatePub = create_publisher<msg::ControllerState>("sp_controller_state", 1);

            mControllerState.names = mActuatorNames;
            mControllerState.states.resize(mActuatorNames.size());
            mControllerState.errors.resize(mActuatorNames.size());
            mControllerState.limits_hit.resize(mActuatorNames.size());

            mControllerState.header.stamp = now();
            mControllerState.header.frame_id = "";
            mControllerState.names = mActuatorNames;
            mControllerState.states.resize(mActuatorNames.size());
            mControllerState.errors.resize(mActuatorNames.size());
            mControllerState.positions.resize(mActuatorNames.size());
            mControllerState.velocities.resize(mActuatorNames.size());
            mControllerState.currents.resize(mActuatorNames.size());
            mControllerState.limits_hit.resize(mActuatorNames.size());
        }

    private:
        std::vector<std::string> const mActuatorNames{"auger", "linear_actuator", "funnel", "brush"};

        std::shared_ptr<ScienceBoard> mScienceBoard;
        std::shared_ptr<BrushedController<Radians>> mAuger;
        std::shared_ptr<BrushedController<Meters>> mLinearActuator;
        std::shared_ptr<Servo> mFunnelServo;
        std::shared_ptr<Servo> mBrushServo;

        Radians mBrushStep;

        rclcpp::CallbackGroup::SharedPtr mServiceGroup;
        rclcpp::Service<srv::ServoPosition>::SharedPtr mFunnelPositionService;
        rclcpp::TimerBase::SharedPtr mPublishDataTimer;
        rclcpp::Subscription<msg::Throttle>::SharedPtr mSPThrottleSub;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr mControllerStatePub;
        msg::ControllerState mControllerState;

        auto processThrottleCmd(msg::Throttle::ConstSharedPtr const& msg) const -> void {
            if (msg->names.size() != msg->throttles.size()) {
                RCLCPP_ERROR(get_logger(), "Name count and value count mismatched!");
                return;
            }

            for (std::size_t i = 0; i < msg->names.size(); ++i) {
                std::string const& name = msg->names[i];
                Dimensionless const& throttle = msg->throttles[i];

                // Silly little thing to save some speed. Could easily just do the straight up string comparision
                switch (name.front() + name.back()) {
                    case 'a' + 'r':
                        mAuger->setDesiredThrottle(throttle);
                        break;
                    case 'l' + 'r':
                        mLinearActuator->setDesiredThrottle(throttle);
                        break;
                    case 'b' + 'h':
                        mBrushServo->setGoalPosition(mBrushServo->getPosition() + mBrushStep);
                        break;
                }
            }
        }

        auto servoPositionCallback(srv::ServoPosition::Request::SharedPtr const& req, srv::ServoPosition::Response::SharedPtr const& res) const -> void {
            if (req->names.size() != 1 || req->names.at(0) != "funnel") return;

            auto const pos = req->positions[0];
            mFunnelServo->setGoalPosition(Radians{pos});
            res->at_tgts.resize(1);
            res->at_tgts[0] = true;
        }

        auto publishDataCallback() -> void {
            mControllerState.header.stamp = now();

            for (std::size_t i = 0; i < mActuatorNames.size(); ++i) {
                std::string_view const name = mActuatorNames[i];

                switch (name.front() + name.back()) {
                    case 'a' + 'r':
                        mControllerState.names[i] = name;
                        mControllerState.states[i] = mAuger->getState();
                        mControllerState.errors[i] = mAuger->getError();
                        mControllerState.positions[i] = mAuger->getPosition();
                        mControllerState.velocities[i] = mAuger->getVelocity();
                        mControllerState.currents[i] = mAuger->getCurrent();
                        mControllerState.limits_hit[i] = mAuger->getLimitsHitBits();
                        break;
                    case 'l' + 'r':
                        mControllerState.names[i] = name;
                        mControllerState.states[i] = mLinearActuator->getState();
                        mControllerState.errors[i] = mLinearActuator->getError();
                        mControllerState.positions[i] = mLinearActuator->getPosition();
                        mControllerState.velocities[i] = mLinearActuator->getVelocity();
                        mControllerState.currents[i] = mLinearActuator->getCurrent();
                        mControllerState.limits_hit[i] = mLinearActuator->getLimitsHitBits();
                        break;
                    case 'f' + 'l':
                        mControllerState.names[i] = name;
                        mControllerState.states[i] = "";
                        mControllerState.errors[i] = "";
                        mControllerState.positions[i] = static_cast<float>(mFunnelServo->getPosition().get());
                        mControllerState.velocities[i] = static_cast<float>(std::numeric_limits<float>::quiet_NaN());
                        mControllerState.currents[i] = static_cast<float>(std::numeric_limits<float>::quiet_NaN());
                        mControllerState.limits_hit[i] = 0u;
                        break;
                    case 'b' + 'h':
                        mControllerState.names[i] = name;
                        mControllerState.states[i] = "";
                        mControllerState.errors[i] = "";
                        mControllerState.positions[i] = static_cast<float>(mBrushServo->getPosition().get());
                        mControllerState.velocities[i] = static_cast<float>(std::numeric_limits<float>::quiet_NaN());
                        mControllerState.currents[i] = static_cast<float>(std::numeric_limits<float>::quiet_NaN());
                        mControllerState.limits_hit[i] = 0u;
                        break;
                }
            }

            mControllerStatePub->publish(mControllerState);
        }
    };

} // namespace mrover


auto main(int const argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto const scienceBridge = std::make_shared<mrover::ScienceHWBridge>();
    scienceBridge->init();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(scienceBridge);
    executor.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
