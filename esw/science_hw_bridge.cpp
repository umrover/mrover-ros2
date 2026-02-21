#include <cstddef>
#include <cstring>
#include <memory>
#include <span>

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/temperature.hpp>
#include <mrover/msg/humidity.hpp>
#include <mrover/msg/pressure.hpp>
#include <mrover/msg/oxygen.hpp>
#include <mrover/msg/ozone.hpp>
#include <mrover/msg/co2.hpp>
#include <mrover/msg/uv.hpp>

#include <science.hpp>
#include <brushed.hpp>


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
        std::vector<std::string> const mActuatorNames{"auger", "linear_actuator"};

        std::shared_ptr<ScienceBoard> mScienceBoard;
        std::shared_ptr<BrushedController<Radians>> mAuger;
        std::shared_ptr<BrushedController<Meters>> mLinearActuator;

        rclcpp::TimerBase::SharedPtr mPublishDataTimer;
        rclcpp::Subscription<msg::Throttle>::SharedPtr mSPThrottleSub;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr mControllerStatePub;
        msg::ControllerState mControllerState;

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
                }
            }

            mControllerStatePub->publish(mControllerState);
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
