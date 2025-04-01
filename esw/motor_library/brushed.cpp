#include "brushed.hpp"

#include <array>

namespace mrover {
    auto BrushedController::updateConfigFromParameters() -> void {
        for (std::size_t i = 0; i < MAX_NUM_LIMIT_SWITCHES; ++i) {
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.present, i, mNode->get_parameter_or(std::format("{}.limit_switch_{}_present", mControllerName, i), false));
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.enabled, i, mNode->get_parameter_or(std::format("{}.limit_switch_{}_enabled", mControllerName, i), false));
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.limits_forward, i, mNode->get_parameter_or(std::format("{}.limit_switch_{}_limits_forward", mControllerName, i), true));
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.active_high, i, mNode->get_parameter_or(std::format("{}.limit_switch_{}_active_high", mControllerName, i), true));
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.use_for_readjustment, i, mNode->get_parameter_or(std::format("{}.limit_switch_{}_used_for_readjustment", mControllerName, i), false));
            mConfigCommand.limit_switch_info.limit_readj_pos.at(i) = OutputPosition{mNode->get_parameter_or(std::format("{}.limit_switch_{}_readjust_position", mControllerName, i), 0.0)};
            mHasLimit |= GET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.present, i);
        }

        mConfigCommand.is_inverted = mNode->get_parameter_or(std::format("{}.is_inverted", mControllerName), false);
        mConfigCommand.gear_ratio = mNode->get_parameter_or(std::format("{}.gear_ratio", mControllerName), 1.0);

        // TODO (ali): put this jawn back and figure out why it fails
        // assert(config.driverVoltage > 0);
        // assert(0 < config.motorMaxVoltage && config.motorMaxVoltage >= config.driverVoltage);
        double const driver_voltage = mNode->get_parameter_or(std::format("{}.driver_voltage", mControllerName), 0.0);
        double const motor_max_voltage = mNode->get_parameter_or(std::format("{}.motor_max_voltage", mControllerName), 12.0);
        assert(driver_voltage > 0);
        assert(motor_max_voltage > 0);
        assert(motor_max_voltage >= driver_voltage);
        mConfigCommand.max_pwm = driver_voltage / motor_max_voltage;

        mConfigCommand.enc_info.quad_present = mNode->get_parameter_or(std::format("{}.quad_present", mControllerName), false);
        mConfigCommand.enc_info.quad_ratio = Ratio{mNode->get_parameter_or(std::format("{}.quad_ratio", mControllerName), 1.0)};

        mConfigCommand.enc_info.abs_present = mNode->get_parameter_or(std::format("{}.abs_present", mControllerName), false);
        mConfigCommand.enc_info.abs_ratio = Ratio{mNode->get_parameter_or(std::format("{}.abs_ratio", mControllerName), 1.0)};
        mConfigCommand.enc_info.abs_offset = OutputPosition{mNode->get_parameter_or(std::format("{}.abs_offset", mControllerName), 0.0)};

        mConfigCommand.min_position = OutputPosition{mNode->get_parameter_or(std::format("{}.min_position", mControllerName), -std::numeric_limits<double>::infinity())};
        mConfigCommand.max_position = OutputPosition{mNode->get_parameter_or(std::format("{}.max_position", mControllerName), std::numeric_limits<double>::infinity())};

        mConfigCommand.min_velocity = OutputVelocity{mNode->get_parameter_or(std::format("{}.min_velocity", mControllerName), -std::numeric_limits<double>::infinity())};
        mConfigCommand.max_velocity = OutputVelocity{mNode->get_parameter_or(std::format("{}.max_velocity", mControllerName), std::numeric_limits<double>::infinity())};

        mPositionGains = Gains{
                .p = mNode->get_parameter_or(std::format("{}.position_p", mControllerName), 0.0),
                .i = mNode->get_parameter_or(std::format("{}.position_i", mControllerName), 0.0),
                .d = mNode->get_parameter_or(std::format("{}.position_d", mControllerName), 0.0),
                .ff = mNode->get_parameter_or(std::format("{}.position_ff", mControllerName), 0.0),
        };
        mVelocityGains = Gains{
                .p = mNode->get_parameter_or(std::format("{}.velocity_p", mControllerName), 0.0),
                .i = mNode->get_parameter_or(std::format("{}.velocity_i", mControllerName), 0.0),
                .d = mNode->get_parameter_or(std::format("{}.velocity_d", mControllerName), 0.0),
                .ff = mNode->get_parameter_or(std::format("{}.velocity_ff", mControllerName), 0.0),
        };

        mCalibrationThrottle = mNode->get_parameter_or(std::format("{}.calibration_throttle", mControllerName), 0.0);
    }

    BrushedController::BrushedController(rclcpp::Node::SharedPtr node, std::string masterName, std::string controllerName)
        : ControllerBase{std::move(node), std::move(masterName), std::move(controllerName)} {

        updateConfigFromParameters();

        mErrorState = "Unknown";
        mState = "Unknown";
    }

    auto BrushedController::setDesiredThrottle(Percent throttle) -> void {
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(throttle >= -1_percent && throttle <= 1_percent);

        mDevice.publish_message(InBoundMessage{ThrottleCommand{.throttle = throttle}});

#ifdef DEBUG_BUILD
        RCLCPP_DEBUG(mNode->get_logger(), "Commanding %s throttle to: %f", mControllerName.c_str(), throttle.rep);
#endif
    }

    auto BrushedController::setDesiredPosition(Radians position) -> void {
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(position >= mConfigCommand.min_position && position <= mConfigCommand.max_position);

        mDevice.publish_message(InBoundMessage{PositionCommand{
                .position = position,
                .p = static_cast<float>(mPositionGains.p),
                .i = static_cast<float>(mPositionGains.i),
                .d = static_cast<float>(mPositionGains.d),
        }});

#ifdef DEBUG_BUILD
        RCLCPP_DEBUG(mNode->get_logger(), "Commanding %s position to: %f", mControllerName.c_str(), position.rep);
#endif
    }

    auto BrushedController::setDesiredVelocity(RadiansPerSecond velocity) -> void {
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(velocity >= mConfigCommand.min_velocity && velocity <= mConfigCommand.max_velocity);

        mDevice.publish_message(InBoundMessage{VelocityCommand{
                .velocity = velocity,
                .p = static_cast<float>(mVelocityGains.p),
                .i = static_cast<float>(mVelocityGains.i),
                .d = static_cast<float>(mVelocityGains.d),
                .ff = static_cast<float>(mVelocityGains.ff),
        }});

#ifdef DEBUG_BUILD
        RCLCPP_DEBUG(mNode->get_logger(), "Commanding %s velocity to: %f", mControllerName.c_str(), velocity.rep);
#endif
    }

    auto BrushedController::sendConfiguration() -> void {
#ifdef DEBUG_BUILD
        RCLCPP_DEBUG(mNode->get_logger(), "Sending configuration to %s", mControllerName.c_str());
#endif
        mDevice.publish_message(InBoundMessage{mConfigCommand});

        // Need to await configuration. Can NOT directly set mIsConfigured to true.
    }

    auto BrushedController::adjust(Radians position) -> void {
        if (!mIsConfigured) {
            sendConfiguration();
            return;
        }

        assert(position >= mConfigCommand.min_position && position <= mConfigCommand.max_position);

        mDevice.publish_message(InBoundMessage{AdjustCommand{.position = position}});

#ifdef DEBUG_BUILD
        RCLCPP_DEBUG(mNode->get_logger(), "Adjusting %s to: %f", mControllerName.c_str(), position.rep);
#endif
    }

    auto BrushedController::processMessage(ControllerDataState const& state) -> void {
        mCurrentPosition = state.position;
        mCurrentVelocity = state.velocity;
        ConfigCalibErrorInfo configCalibErrInfo = state.config_calib_error_data;
        mIsConfigured = configCalibErrInfo.configured;
        mIsCalibrated = configCalibErrInfo.calibrated;
        mErrorState = errorToString(configCalibErrInfo.error);
        auto const& [_, hit] = state.limit_switches;
        for (std::size_t i = 0; i < mLimitHit.size(); ++i) {
            mLimitHit.at(i) = GET_BIT_AT_INDEX(hit, i);
        }
        if (mIsCalibrated) {
            mState = "Armed";
        } else if (mIsConfigured) {
            mState = "Not Calibrated";
        } else {
            mState = "Not Configured";
        }
    }

    auto BrushedController::processCANMessage(msg::CAN::ConstSharedPtr const& msg) -> void {
        assert(msg->source == mControllerName);
        assert(msg->destination == mMasterName);

        OutBoundMessage const& message = *reinterpret_cast<OutBoundMessage const*>(msg->data.data());

        // This calls the correct process function based on the current value of the alternative
        std::visit([&](auto const& messageAlternative) { processMessage(messageAlternative); }, message);
    }

    auto BrushedController::errorToString(BDCMCErrorInfo errorCode) -> std::string {
        switch (errorCode) {
            case BDCMCErrorInfo::NO_ERROR:
                return "NO_ERROR";
            case BDCMCErrorInfo::DEFAULT_START_UP_NOT_CONFIGURED:
                return "DEFAULT_START_UP_NOT_CONFIGURED";
            case BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED:
                return "RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED";
            case BDCMCErrorInfo::RECEIVING_POSITION_COMMANDS_WHEN_NOT_CALIBRATED:
                return "RECEIVING_POSITION_COMMANDS_WHEN_NOT_CALIBRATED";
            case BDCMCErrorInfo::OUTPUT_SET_TO_ZERO_SINCE_EXCEEDING_LIMITS:
                return "OUTPUT_SET_TO_ZERO_SINCE_EXCEEDING_LIMITS";
            case BDCMCErrorInfo::RECEIVING_PID_COMMANDS_WHEN_NO_READER_EXISTS:
                return "RECEIVING_PID_COMMANDS_WHEN_NO_READER_EXISTS";
            default:
                return "UNKNOWN_ERROR_CODE";
        }
    }

    // TODO:(owen) ROS1 returned bool and now we don't anymore? idk
    auto BrushedController::calibrateServiceCallback([[maybe_unused]] std_srvs::srv::Trigger::Request::SharedPtr const req, std_srvs::srv::Trigger::Response::SharedPtr res) -> void {
        if (!mHasLimit) {
            res->success = false;
            res->message = std::format("{} does not have limit switches, cannot calibrate", mControllerName);
            // return true;
            return;
        }
        if (mIsCalibrated) {
            res->success = false;
            res->message = std::format("{} already calibrated", mControllerName);
            // return true;
            return;
        }
        // sends throttle command until a limit switch is hit
        // mIsCalibrated is set with CAN message coming from BDCMC
        setDesiredThrottle(mCalibrationThrottle);
        res->success = true;
    }

} // namespace mrover
