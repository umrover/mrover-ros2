#include "brushed.hpp"

#include <array>

namespace mrover {
    auto BrushedController::updateConfigFromParameters() -> void {
        bool isInverted;
        double gearRatio, driverVoltage, motorMaxVoltage;
        bool quadPresent, absPresent;
        double quadRatio, absRatio, absOffset;
        double minPosition, maxPosition;
        double minVelocity, maxVelocity;
        double calibrationThrottle;
        std::array<bool, MAX_NUM_LIMIT_SWITCHES> limitSwitchesPresent{};
        std::array<bool, MAX_NUM_LIMIT_SWITCHES> limitSwitchesEnabled{};
        std::array<bool, MAX_NUM_LIMIT_SWITCHES> limitSwitchesLimitsForward{};
        std::array<bool, MAX_NUM_LIMIT_SWITCHES> limitSwitchesActiveHigh{};
        std::array<bool, MAX_NUM_LIMIT_SWITCHES> limitSwitchesUsedForReadjustment{};
        std::array<double, MAX_NUM_LIMIT_SWITCHES> limitSwitchesReadjustPosition{};

        std::vector<ParameterWrapper> parameters = {
                {std::format("{}.is_inverted", mControllerName), isInverted, false},
                {std::format("{}.gear_ratio", mControllerName), gearRatio, 1.0},
                {std::format("{}.driver_voltage", mControllerName), driverVoltage, 0.0},
                {std::format("{}.motor_max_voltage", mControllerName), motorMaxVoltage, 12.0},
                {std::format("{}.quad_present", mControllerName), quadPresent, false},
                {std::format("{}.quad_ratio", mControllerName), quadRatio, 1.0},
                {std::format("{}.abs_present", mControllerName), absPresent, false},
                {std::format("{}.abs_ratio", mControllerName), absRatio, 1.0},
                {std::format("{}.abs_offset", mControllerName), absOffset, 0.0},
                {std::format("{}.min_position", mControllerName), minPosition, -std::numeric_limits<double>::infinity()},
                {std::format("{}.max_position", mControllerName), maxPosition, std::numeric_limits<double>::infinity()},
                {std::format("{}.min_velocity", mControllerName), minVelocity, -std::numeric_limits<double>::infinity()},
                {std::format("{}.max_velocity", mControllerName), maxVelocity, std::numeric_limits<double>::infinity()},
                {std::format("{}.calibration_throttle", mControllerName), calibrationThrottle, 0.0},
                {std::format("{}.position_p", mControllerName), mPositionGains.p, 0.0},
                {std::format("{}.position_i", mControllerName), mPositionGains.i, 0.0},
                {std::format("{}.position_d", mControllerName), mPositionGains.d, 0.0},
                {std::format("{}.position_ff", mControllerName), mPositionGains.ff, 0.0},
                {std::format("{}.velocity_p", mControllerName), mVelocityGains.p, 0.0},
                {std::format("{}.velocity_i", mControllerName), mVelocityGains.i, 0.0},
                {std::format("{}.velocity_d", mControllerName), mVelocityGains.d, 0.0},
                {std::format("{}.velocity_ff", mControllerName), mVelocityGains.ff, 0.0},
        };
        for (std::size_t i = 0; i < MAX_NUM_LIMIT_SWITCHES; ++i) {
            parameters.emplace_back(std::format("{}.limit_switch_{}_present", mControllerName, i), limitSwitchesPresent[i], false);
            parameters.emplace_back(std::format("{}.limit_switch_{}_enabled", mControllerName, i), limitSwitchesEnabled[i], true);
            parameters.emplace_back(std::format("{}.limit_switch_{}_limits_forward", mControllerName, i), limitSwitchesLimitsForward[i], false);
            parameters.emplace_back(std::format("{}.limit_switch_{}_active_high", mControllerName, i), limitSwitchesActiveHigh[i], true);
            parameters.emplace_back(std::format("{}.limit_switch_{}_used_for_readjustment", mControllerName, i), limitSwitchesUsedForReadjustment[i], false);
            parameters.emplace_back(std::format("{}.limit_switch_{}_readjust_position", mControllerName, i), limitSwitchesReadjustPosition[i], 0.0);
        }

        ParameterWrapper::declareParameters(mNode.get(), parameters);

        mConfigCommand.is_inverted = isInverted;
        mConfigCommand.gear_ratio = gearRatio;

        assert(driverVoltage > 0);
        assert(motorMaxVoltage > 0);
        assert(motorMaxVoltage >= driverVoltage);
        mConfigCommand.max_pwm = driverVoltage / motorMaxVoltage;

        mConfigCommand.enc_info.quad_present = quadPresent;
        mConfigCommand.enc_info.quad_ratio = Ratio{quadRatio};

        mConfigCommand.enc_info.abs_present = absPresent;
        mConfigCommand.enc_info.abs_ratio = Ratio{absRatio};
        mConfigCommand.enc_info.abs_offset = OutputPosition{absOffset};

        mConfigCommand.min_position = OutputPosition{minPosition};
        mConfigCommand.max_position = OutputPosition{maxPosition};

        mConfigCommand.min_velocity = OutputVelocity{minVelocity};
        mConfigCommand.max_velocity = OutputVelocity{maxVelocity};

        mCalibrationThrottle = calibrationThrottle;

        for (std::size_t i = 0; i < MAX_NUM_LIMIT_SWITCHES; ++i) {
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.present, i, limitSwitchesPresent[i]);
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.enabled, i, limitSwitchesEnabled[i]);
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.limits_forward, i, limitSwitchesLimitsForward[i]);
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.active_high, i, limitSwitchesActiveHigh[i]);
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.use_for_readjustment, i, limitSwitchesUsedForReadjustment[i]);
            mConfigCommand.limit_switch_info.limit_readj_pos.at(i) = OutputPosition{limitSwitchesReadjustPosition[i]};
            mHasLimit |= GET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.present, i);
        }
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
