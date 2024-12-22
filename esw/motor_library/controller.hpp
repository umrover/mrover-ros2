#pragma once

#include <chrono>
#include <string>

#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <can_device.hpp>
#include <units/units.hpp>

#include <mrover/msg/can.hpp>
#include <mrover/msg/controller_state.hpp>
#include <mrover/msg/motors_adjust.hpp>
#include <mrover/msg/position.hpp>
#include <mrover/msg/throttle.hpp>
#include <mrover/msg/velocity.hpp>
#include <mrover/srv/adjust_motor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace mrover {

    // This uses CRTP to allow for static polymorphism
    // See: https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern
    template<IsUnit TOutputPosition, typename Derived>
    class ControllerBase {
    public:
        using OutputPosition = TOutputPosition;
        using OutputVelocity = compound_unit<OutputPosition, inverse<Seconds>>;

        ControllerBase(rclcpp::Node::SharedPtr node, std::string masterName, std::string controllerName)
            : mNode{std::move(node)},
              mMasterName{std::move(masterName)},
              mControllerName{std::move(controllerName)},
              mDevice{mNode->shared_from_this(), mMasterName, mControllerName} {

            mIncomingCANSub = mNode->create_subscription<msg::CAN>(
                    std::format("can/{}/in", mControllerName), 16,
                    [this](msg::CAN::ConstSharedPtr const& msg) { processCANMessage(msg); });

            mJointDataPub = mNode->create_publisher<sensor_msgs::msg::JointState>(
                    std::format("{}_joint_data", mControllerName), 1);
            mControllerDataPub = mNode->create_publisher<msg::ControllerState>(
                    std::format("{}_controller_data", mControllerName), 1);

            mAdjustServer = mNode->create_service<srv::AdjustMotor>(
                    std::format("{}_adjust", mControllerName),
                    [this](srv::AdjustMotor::Request::SharedPtr const req,
                           srv::AdjustMotor::Response::SharedPtr res) {
                        adjustServiceCallback(req, res);
                    });
        }

        ControllerBase(ControllerBase const&) = delete;
        ControllerBase(ControllerBase&&) = delete;

        auto operator=(ControllerBase const&) -> ControllerBase& = delete;
        auto operator=(ControllerBase&&) -> ControllerBase& = delete;

        // TODO:(quintin) Why can't I bind directly to &Derived::processCANMessage?
        // TODO:(owen) that's a great question
        auto processCANMessage(msg::CAN::ConstSharedPtr const& msg) -> void {
            static_cast<Derived*>(this)->processCANMessage(msg);
        }

        [[nodiscard]] auto isJointDe() const -> bool {
            return mControllerName == "joint_de_0" || mControllerName == "joint_de_1";
        }

        auto adjustServiceCallback(srv::AdjustMotor::Request::SharedPtr const req, srv::AdjustMotor::Response::SharedPtr res) -> void {
            if (req->name != mControllerName) {
                RCLCPP_ERROR(mNode->get_logger(), "Adjust request at server for %s ignored", req->name.c_str());
                res->success = false;
                return;
            }
            using Position = typename detail::strip_conversion<OutputPosition>::type;
            OutputPosition position = Position{req->value};
            static_cast<Derived*>(this)->adjust(position);
            res->success = true;
        }

        [[nodiscard]] auto getPosition() const -> OutputPosition {
            return mCurrentPosition;
        }
        [[nodiscard]] auto getVelocity() const -> OutputVelocity {
            return mCurrentVelocity;
        }
        [[nodiscard]] auto getEffort() const -> double {
            return mCurrentEffort;
        }

        [[nodiscard]] auto getErrorState() const -> std::string {
            return mErrorState;
        }
        [[nodiscard]] auto getState() const -> std::string {
            return mState;
        }
        [[nodiscard]] auto getLimitsHitBits() const -> std::uint8_t {
            std::uint8_t limitsHit{};
            for (int i = 0; i < 4; ++i) {
                limitsHit |= mLimitHit.at(i) << i;
            }
            return limitsHit;
        }


    protected:
        rclcpp::Node::SharedPtr mNode;
        std::string mMasterName, mControllerName;
        CanDevice mDevice;
        rclcpp::Subscription<msg::CAN>::SharedPtr mIncomingCANSub;
        OutputPosition mCurrentPosition{};
        OutputVelocity mCurrentVelocity{};
        double mCurrentEffort{std::numeric_limits<double>::quiet_NaN()};
        Percent mCalibrationThrottle{};
        bool mIsCalibrated{};
        bool mHasLimit{};
        std::string mErrorState;
        std::string mState;
        std::array<bool, 4> mLimitHit{};

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mJointDataPub;
        rclcpp::Publisher<msg::ControllerState>::SharedPtr mControllerDataPub;

        rclcpp::Service<srv::AdjustMotor>::SharedPtr mAdjustServer;
    };

} // namespace mrover
