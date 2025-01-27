#include "brushed.hpp"

#include <array>

namespace mrover {
    BrushedController::BrushedController(rclcpp::Node::SharedPtr node, std::string masterName, std::string controllerName, Config config)
        : ControllerBase{std::move(node), std::move(masterName), std::move(controllerName)} {

        for (std::size_t i = 0; i < Config::MAX_NUM_LIMIT_SWITCHES; ++i) {
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.present, i, config.limitSwitchPresent[i]);
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.enabled, i, config.limitSwitchEnabled[i]);
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.limits_forward, i, config.limitSwitchLimitsFwd[i]);
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.active_high, i, config.limitSwitchActiveHigh[i]); // might switch default value to false depending on wiring
            SET_BIT_AT_INDEX(mConfigCommand.limit_switch_info.use_for_readjustment, i, config.limitSwitchUsedForReadjustment[i]);
            mConfigCommand.limit_switch_info.limit_readj_pos.at(i) = config.limitSwitchReadjustPosition[i];
        }
        mConfigCommand.limit_switch_info.limit_max_forward_position = config.limitMaxForwardPosition;
        mConfigCommand.limit_switch_info.limit_max_backward_position = config.limitMaxBackwardPosition;

        mConfigCommand.is_inverted = config.isInverted;
        mConfigCommand.gear_ratio = config.gearRatio;

        // TODO (ali): put this jawn back and figure out why it fails
        // assert(config.driverVoltage > 0);
        // assert(0 < config.motorMaxVoltage && config.motorMaxVoltage >= config.driverVoltage);
        mConfigCommand.max_pwm = config.motorMaxVoltage / config.driverVoltage;

        mConfigCommand.enc_info.quad_present = config.quadPresent;
        mConfigCommand.enc_info.quad_ratio = config.quadRatio;

        mConfigCommand.enc_info.abs_present = config.absPresent;
        mConfigCommand.enc_info.abs_ratio = config.absRatio;
        mConfigCommand.enc_info.abs_offset = config.absOffset;

        mConfigCommand.min_position = config.minPosition;
        mConfigCommand.max_position = config.maxPosition;

        mConfigCommand.min_velocity = config.minVelocity;
        mConfigCommand.max_velocity = config.maxVelocity;

        mPositionGains = config.positionGains;
        mVelocityGains = config.velocityGains;

        for (std::size_t i = 0; i < Config::MAX_NUM_LIMIT_SWITCHES; ++i) {
            mHasLimit |= config.limitSwitchEnabled[i];
        }
        mCalibrationThrottle = config.calibrationThrottle;
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
    }

    auto BrushedController::sendConfiguration() -> void {
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
