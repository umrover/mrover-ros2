#include <cstddef>
#include <cstring>
#include <memory>
#include <span>
#include <unordered_map>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mrover/msg/co2.hpp>
#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/humidity.hpp>
#include <mrover/msg/oxygen.hpp>
#include <mrover/msg/ozone.hpp>
#include <mrover/msg/pressure.hpp>
#include <mrover/msg/temperature.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/uv.hpp>

#include <mrover/srv/servo_position.hpp>

#include <brushed.hpp>
#include <science.hpp>
#include <servo.hpp>


namespace mrover {
    class ScienceHWBridge : public rclcpp::Node {

        using Controller = std::variant<
                std::shared_ptr<BrushedController<Meters>>,
                std::shared_ptr<BrushedController<Radians>>>;

    public:
        ScienceHWBridge() : Node{"science_hw_bridge"} {}

        auto init() -> void {
            mScienceBoard = std::make_shared<ScienceBoard>(shared_from_this(), "jetson", "science");
            mAuger = std::make_shared<BrushedController<Radians>>(shared_from_this(), "jetson", "auger");
            mLinearActuator = std::make_shared<BrushedController<Meters>>(shared_from_this(), "jetson", "linear_actuator");

            mSPThrottleSub = create_subscription<msg::Throttle>("sp_thr_cmd", 1, [this](msg::Throttle::ConstSharedPtr const& msg) { processThrottleCmd(msg); });

            mPublishDataTimer = create_wall_timer(
                    std::chrono::milliseconds(100),
                    [this]() -> void { publishDataCallback(); });
            mControllerStatePub = create_publisher<msg::ControllerState>("sp_controller_state", 1);

            std::vector<ParameterWrapper> parameters = {
                {"u2d2_device", mU2D2Device, "/dev/USB0"},
            };
            ParameterWrapper::declareParameters(this, parameters);

            Servo::init(mU2D2Device);
            for (auto const& servo: mServoNames) {
                create_servo(servo.second, servo.first);
            }

            mPositionService = create_service<mrover::srv::ServoPosition>("sp_funnel_servo", [this](mrover::srv::ServoPosition::Request::SharedPtr const& request, mrover::srv::ServoPosition::Response::SharedPtr const& response){ positionServiceCallback(request, response); });

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
        std::string mU2D2Device;
        rclcpp::Service<mrover::srv::ServoPosition>::SharedPtr mPositionService;
        std::unordered_map<std::string, std::shared_ptr<mrover::Servo>> mServos;
        std::vector<std::pair<std::string, int>> const mServoNames = {{"funnel", 5}};

        std::vector<std::string> const mActuatorNames{"auger", "linear_actuator", "funnel"};

        std::shared_ptr<ScienceBoard> mScienceBoard;
        std::shared_ptr<BrushedController<Radians>> mAuger;
        std::shared_ptr<BrushedController<Meters>> mLinearActuator;

        rclcpp::TimerBase::SharedPtr mPublishDataTimer;
        rclcpp::Subscription<msg::Throttle>::SharedPtr mSPThrottleSub;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr mControllerStatePub;
        msg::ControllerState mControllerState;

        auto create_servo(uint8_t id, std::string const& name) -> void {
            mServos.insert({name, std::make_shared<mrover::Servo>(shared_from_this(), id, name)});
        }

        static auto servos_active(std::vector<Servo::ServoStatus>& statuses) -> bool {
            bool active = false;
            for (auto& status: statuses) {
                if (status == Servo::ServoStatus::Active) {
                    active = true;
                }
            }
            return active;
        }

        auto positionServiceCallback(mrover::srv::ServoPosition::Request::SharedPtr const& request, mrover::srv::ServoPosition::Response::SharedPtr const& response) -> void {
            size_t const n = request->names.size();
            std::vector<Servo::ServoStatus> statuses = std::vector<Servo::ServoStatus>(n);

            auto const timeout = std::chrono::seconds(3);
            auto const start = this->get_clock()->now();

            for (size_t i = 0; i < n; i++) {
                statuses[i] = mServos.at(request->names[i])->setPosition(request->positions[i], Servo::ServoMode::Limited);
            }

            while (servos_active(statuses)) {
                for (size_t i = 0; i < n; i++) {
                    statuses[i] = mServos.at(request->names[i])->getTargetStatus();
                    response->at_tgts[i] = (statuses[i] == Servo::ServoStatus::Success);
                }
                if (this->get_clock()->now() - start > timeout) {
                    RCLCPP_WARN(this->get_logger(), "Timeout reached while waiting for servo to reach target position");
                    break;
                }
            }
        }

        auto processThrottleCmd(msg::Throttle::ConstSharedPtr const& msg) -> void {
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
                    case 'f' + 'l':
                        break;
                }
            }
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
                        packServoState(name, i);
                        break;
                }
            }

            mControllerStatePub->publish(mControllerState);
        }

        auto packServoState(std::string_view name, size_t i) -> void {
            auto& servo = *mServos.at(std::string(name));
            mControllerState.names[i] = name;
            mControllerState.limits_hit[i] = servo.getLimitStatus();

            float pos = 0.0f;
            float vel = 0.0f;
            float cur = 0.0f;

            Servo::ServoStatus err = servo.getPosition(pos);
            servo.getVelocity(vel);
            servo.getCurrent(cur);

            mControllerState.positions[i] = pos;
            mControllerState.velocities[i] = vel;
            mControllerState.currents[i] = cur;

            Servo::ServoStatus ts = servo.getTargetStatus();

            /*mControllerState.state[i] = {servo->getState()};*/
            if (ts == Servo::ServoStatus::Active) {
                mControllerState.states[i] = "active";
            } else if (ts == Servo::ServoStatus::Success) {
                mControllerState.states[i] = "success";
            } else {
                mControllerState.states[i] = "error";
            }

            /*mControllerState.error[i] = {servo->getErrorState()};*/
            switch (err) {
                case Servo::ServoStatus::CommNotAvailable:
                    mControllerState.errors[i] = "CommNotAvailable";
                    break;
                case Servo::ServoStatus::CommTxError:
                    mControllerState.errors[i] = "CommTxError";
                    break;
                case Servo::ServoStatus::HardwareFailure:
                    mControllerState.errors[i] = "HardwareFailure";
                    break;
                case Servo::ServoStatus::CommRxCorrupt:
                    mControllerState.errors[i] = "CommRxCorrupt";
                    break;
                case Servo::ServoStatus::CommRxTimeout:
                    mControllerState.errors[i] = "CommRxTimeout";
                    break;
                case Servo::ServoStatus::CommRxFail:
                    mControllerState.errors[i] = "CommRxFail";
                    break;
                case Servo::ServoStatus::CommTxFail:
                    mControllerState.errors[i] = "CommTxFail";
                    break;
                case Servo::ServoStatus::CommPortBusy:
                    mControllerState.errors[i] = "CommPortBusy";
                    break;
                case Servo::ServoStatus::CommRxWaiting:
                    mControllerState.errors[i] = "CommRxWaiting";
                    break;
                case Servo::ServoStatus::FailedToOpenPort:
                    mControllerState.errors[i] = "FailedToOpenPort";
                    break;
                case Servo::ServoStatus::FailedToSetBaud:
                    mControllerState.errors[i] = "FailedToSetBaud";
                    break;
                default:
                    mControllerState.errors[i] = "NoError";
                    break;
            }
        }

    };

} // namespace mrover


auto main(int argc, char** argv) -> int {
    rclcpp::init(argc, argv);
    auto scienceBridge = std::make_shared<mrover::ScienceHWBridge>();
    scienceBridge->init();
    rclcpp::spin(scienceBridge);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
