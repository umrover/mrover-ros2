#pragma once

#include "mrover/msg/detail/controller_state__struct.hpp"
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <string>

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>

#include <can_device.hpp>
#include <chrono>
#include <units/units.hpp>

#include <sensor_msgs/msg/joint_state.h>

#include <mrover/srv/adjust_motor.hpp>
#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/motors_adjust.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>
#include <mrover/msg/can.hpp>

namespace mrover {

    // This uses CRTP to allow for static polymorphism
    // See: https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern
    template<IsUnit TOutputPosition, typename Derived>
    class ControllerBase : public rclcpp::Node {
    public:
        using OutputPosition = TOutputPosition;
        using OutputVelocity = compound_unit<OutputPosition, inverse<Seconds>>;

        ControllerBase(std::string masterName, std::string controllerName) : rclcpp::Node{NODE_NAME}, mMasterName{std::move(masterName)}, mControllerName{std::move(controllerName)} {
            try {   
                mIncomingCANSub = create_subscription<msg::CAN>(std::format("can/{}/in", mControllerName), 16, &ControllerBase::processCANMessage);
                mMoveThrottleSub = create_subscription<msg::Throttle>(std::format("{}_throttle_cmd", mControllerName), 1, &ControllerBase::setDesiredThrottle);
                mMoveVelocitySub = create_subscription<msg::Velocity>(std::format("{}_velocity_cmd", mControllerName), 1, &ControllerBase::setDesiredVelocity);
                mMovePositionSub = create_subscription<msg::Position>(std::format("{}_position_cmd", mControllerName), 1, &ControllerBase::setDesiredPosition);
                mJointDataPub = create_publisher<sensor_msgs::msg::JointState>(std::format("{}_joint_data", mControllerName), 1);
                mPublishDataTimer = rclcpp::create_wall_timer(std::chrono::milliseconds(100), std::bind(&ControllerBase::publishDataCallback));
                mControllerDataPub = create_publisher<msg::ControllerState>(std::format("{}_controller_data", mControllerName), 1);
                // mAdjustServer{mNh.advertiseService(std::format("{}_adjust", mControllerName), &ControllerBase::adjustServiceCallback, this)};
            } catch (std::exception const& e) {
                RCLCPP_ERROR_STREAM(get_logger(), std::format("Exception initializing: {}", e.what()));
                rclcpp::shutdown();
            }
        }

        ControllerBase(ControllerBase const&) = delete;
        ControllerBase(ControllerBase&&) = delete;

        auto operator=(ControllerBase const&) -> ControllerBase& = delete;
        auto operator=(ControllerBase&&) -> ControllerBase& = delete;

        // TODO(quintin): Why can't I bind directly to &Derived::processCANMessage?
        auto processCANMessage(msg::CAN::ConstPtr const& msg) -> void {
            static_cast<Derived*>(this)->processCANMessage(msg);
        }

        auto setDesiredThrottle(msg::Throttle::ConstPtr const& msg) -> void {
            if (msg->names.size() != 1 || msg->names.at(0) != mControllerName || msg->throttles.size() != 1) {
                RCLCPP_ERROR(get_logger(), "Throttle request at topic for %s ignored!", msg->names.at(0).c_str());
                return;
            }

            static_cast<Derived*>(this)->setDesiredThrottle(msg->throttles.front());
        }

        auto setDesiredVelocity(msg::Velocity::ConstPtr const& msg) -> void {
            if (msg->names.size() != 1 || msg->names.at(0) != mControllerName || msg->velocities.size() != 1) {
                RCLCPP_ERROR(get_logger(), "Velocity request at topic for %s ignored!", msg->names.at(0).c_str());
                return;
            }

            // ROS message will always be in SI units with no conversions
            using Velocity = typename detail::strip_conversion<OutputVelocity>::type;
            OutputVelocity velocity = Velocity{msg->velocities.front()};
            static_cast<Derived*>(this)->setDesiredVelocity(velocity);
        }

        auto setDesiredPosition(msg::Position::ConstPtr const& msg) -> void {
            if (msg->names.size() != 1 || msg->names.at(0) != mControllerName || msg->positions.size() != 1) {
                RCLCPP_ERROR(get_logger(), "Position request at topic for %s ignored!", msg->names.at(0).c_str());
                return;
            }

            // ROS message will always be in SI units with no conversions
            using Position = typename detail::strip_conversion<OutputPosition>::type;
            OutputPosition position = Position{msg->positions.front()};
            static_cast<Derived*>(this)->setDesiredPosition(position);
        }

        [[nodiscard]] auto isJointDe() const -> bool {
            return mControllerName == "joint_de_0" || mControllerName == "joint_de_1";
        }

        auto publishDataCallback() -> void {
            {
                using Position = typename detail::strip_conversion<OutputPosition>::type;
                using Velocity = typename detail::strip_conversion<OutputVelocity>::type;

                sensor_msgs::msg::JointState jointState;
                jointState.header.stamp = get_clock()->now();
                jointState.name = {mControllerName};
                jointState.position = {Position{mCurrentPosition}.get()};
                jointState.velocity = {Velocity{mCurrentVelocity}.get()};
                jointState.effort = {static_cast<Derived*>(this)->getEffort()};
                mJointDataPub->publish(jointState);
            }
            {
                msg::ControllerState controllerState;
                controllerState.name = {mControllerName};
                controllerState.state = {mState};
                controllerState.error = {mErrorState};
                std::uint8_t limit_hit{};
                for (int i = 0; i < 4; ++i) {
                    limit_hit |= mLimitHit.at(i) << i;
                }
                controllerState.limit_hit = {limit_hit};

                mControllerDataPub.publish(controllerState);
            }
        }

        auto adjustServiceCallback(AdjustMotor::Request& req, AdjustMotor::Response& res) -> bool {
            if (req.name != mControllerName) {
                RCLCPP_ERROR(get_logger(), "Adjust request at server for %s ignored", req.name.c_str());
                res.success = false;
                return true;
            }

            using Position = typename detail::strip_conversion<OutputPosition>::type;
            OutputPosition position = Position{req.value};
            static_cast<Derived*>(this)->adjust(position);
            res.success = true;
            return true;
        }

    protected:
        static constexpr char const* NODE_NAME = "motor_controller";

        std::string mMasterName, mControllerName;
        CanDevice mDevice;
        OutputPosition mCurrentPosition{};
        OutputVelocity mCurrentVelocity{};
        Percent mCalibrationThrottle{};
        bool mIsCalibrated{};
        bool mHasLimit{};
        std::string mErrorState;
        std::string mState;
        std::array<bool, 4> mLimitHit{};
        

        rclcpp::Subscription<msg::CAN> mIncomingCANSub;
        rclcpp::Subscription<msg::Throttle>::SharedPtr mMoveThrottleSub;
        rclcpp::Subscription<msg::Velocity>::SharedPtr mMoveVelocitySub;
        rclcpp::Subscription<msg::Position>::SharedPtr mMovePositionSub;
        rclcpp::Publisher<msg::Position>::SharedPtr mJointDataPub;
        rclcpp::Publisher<msg::ControllerState> mControllerDataPub;
        rclcpp::TimerBase::SharedPtr mPublishDataTimer;
        // ros::ServiceServer mAdjustServer;
    };

} // namespace mrover
