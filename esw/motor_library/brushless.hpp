#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <moteus/moteus.h>
#include <moteus/moteus_multiplex.h>

#include <can_device.hpp>
#include <controller.hpp>
#include <parameter.hpp>

namespace mrover {

    using namespace mjbots;

    enum class Mode {
        Stopped = 0,
        Fault = 1,
        PreparingToOperate1 = 2,
        PreparingToOperate2 = 3,
        PreparingToOperate3 = 4,
        PWMMode = 5,
        VoltageMode = 6,
        VoltageFOC = 7,
        VoltageDQ = 8,
        Current = 9,
        Position = 10,
        Timeout = 11,
        ZeroVelocity = 12,
        StayWithin = 13,
        MeasureInductance = 14,
        Brake = 15
    };

    enum class ErrorCode {
        DmaStreamTransferError = 1,
        DmaStreamFifiError = 2,
        UartOverrunError = 3,
        UartFramingError = 4,
        UartNoiseError = 5,
        UartBufferOverrunError = 6,
        UartParityError = 7,
        CalibrationFault = 32,
        MotorDriverFault = 33,
        OverVoltage = 34,
        EncoderFault = 35,
        MotorNotConfigured = 36,
        PwmCycleOverrun = 37,
        OverTemperature = 38,
        StartOutsideLimit = 39,
        UnderVoltage = 40,
        ConfigChanged = 41,
        ThetaInvalid = 42,
        PositionInvalid = 43,
    };

    struct MoteusLimitSwitchInfo {
        bool isForwardPressed{};
        bool isBackwardPressed{};
    };

    template<IsUnit TOutputPosition>
    class BrushlessController final : public ControllerBase<TOutputPosition, BrushlessController<TOutputPosition>> {
        using Base = ControllerBase<TOutputPosition, BrushlessController>;

        // TODO(quintin): this is actually so dumb
        using OutputPosition = typename Base::OutputPosition;
        using OutputVelocity = typename Base::OutputVelocity;

        using Base::mControllerName;
        using Base::mCurrentEffort;
        using Base::mCurrentPosition;
        using Base::mCurrentVelocity;
        using Base::mDevice;
        using Base::mErrorState;
        using Base::mLimitHit;
        using Base::mMasterName;
        using Base::mNode;
        using Base::mState;

        enum class MoteusAuxNumber : int {
            AUX1 = 1,
            AUX2 = 2,
        };

        enum class MoteusAuxPin : int {
            PIN0 = 0,
            PIN1 = 1,
            PIN2 = 2,
            PIN3 = 3,
            PIN4 = 4,
        };

        struct LimitSwitchInfo {
            bool present = false;
            bool enabled = true;
            bool limitsForward = false;
            bool activeHigh = true;
            bool usedForReadjustment = false;
            OutputPosition readjustPosition = OutputPosition{0.0};
            MoteusAuxNumber auxNumber = MoteusAuxNumber::AUX1;
            MoteusAuxPin auxPin = MoteusAuxPin::PIN0;
        };

        constexpr static std::size_t MAX_NUM_LIMIT_SWITCHES = 2;
        static_assert(MAX_NUM_LIMIT_SWITCHES <= 2, "Only 2 limit switches are supported");

        OutputVelocity mMinVelocity = OutputVelocity{-1.0};
        OutputVelocity mMaxVelocity = OutputVelocity{1.0};
        OutputPosition mMinPosition = OutputPosition{-1.0};
        OutputPosition mMaxPosition = OutputPosition{1.0};
        double mMaxTorque = 0.3;
        double mWatchdogTimeout = 0.25;
        std::array<LimitSwitchInfo, MAX_NUM_LIMIT_SWITCHES> mLimitSwitchesInfo{};

        std::optional<moteus::Controller> mMoteus;
        std::int8_t mMoteusAux1Info{}, mMoteusAux2Info{};
        bool mHasLimit{};

    public:
        BrushlessController(rclcpp::Node::SharedPtr node, std::string masterName, std::string controllerName)
            : Base{std::move(node), std::move(masterName), std::move(controllerName)} {

            double minVelocity, maxVelocity;
            double minPosition, maxPosition;
            std::vector<ParameterWrapper> parameters = {
                    {std::format("{}.min_velocity", mControllerName), minVelocity, -1.0},
                    {std::format("{}.max_velocity", mControllerName), maxVelocity, 1.0},
                    {std::format("{}.min_position", mControllerName), minPosition, -1.0},
                    {std::format("{}.max_position", mControllerName), maxPosition, 1.0},
                    {std::format("{}.max_torque", mControllerName), mMaxTorque, 0.3},
                    {std::format("{}.watchdog_timeout", mControllerName), mWatchdogTimeout, 0.25},
            };

            for (std::size_t i = 0; i < MAX_NUM_LIMIT_SWITCHES; ++i) {
                parameters.emplace_back(std::format("{}.limit_switch_{}_present", mControllerName, i), mLimitSwitchesInfo[i].present, false);
                parameters.emplace_back(std::format("{}.limit_switch_{}_enabled", mControllerName, i), mLimitSwitchesInfo[i].enabled, true);
                parameters.emplace_back(std::format("{}.limit_switch_{}_limits_forward", mControllerName, i), mLimitSwitchesInfo[i].limitsForward, false);
                parameters.emplace_back(std::format("{}.limit_switch_{}_active_high", mControllerName, i), mLimitSwitchesInfo[i].activeHigh, true);
                parameters.emplace_back(std::format("{}.limit_switch_{}_used_for_readjustment", mControllerName, i), mLimitSwitchesInfo[i].usedForReadjustment, false);
                parameters.emplace_back(std::format("{}.limit_switch_{}_readjust_position", mControllerName, i), mLimitSwitchesInfo[i].readjustPosition.rep, 0.0);
                parameters.emplace_back(std::format("{}.limit_switch_{}_aux_number", mControllerName, i), mLimitSwitchesInfo[i].auxNumber, MoteusAuxNumber::AUX1);
                parameters.emplace_back(std::format("{}.limit_switch_{}_aux_pin", mControllerName, i), mLimitSwitchesInfo[i].auxPin, MoteusAuxPin::PIN0);
            }

            ParameterWrapper::declareParameters(mNode.get(), parameters);

            mMinVelocity = OutputVelocity{minVelocity};
            mMaxVelocity = OutputVelocity{maxVelocity};
            mMinPosition = OutputPosition{minPosition};
            mMaxPosition = OutputPosition{maxPosition};

            // if active low, we want to make the default value make it believe that
            // the limit switch is NOT pressed.
            // This is because we may not receive the newest query message from the moteus
            // as a result of either testing or startup.
            for (std::size_t i = 0; i < MAX_NUM_LIMIT_SWITCHES; ++i) {
                if (mLimitSwitchesInfo[i].present && mLimitSwitchesInfo[i].enabled) {
                    mHasLimit = true;
                }

                if (mLimitSwitchesInfo[i].present && mLimitSwitchesInfo[i].enabled && !mLimitSwitchesInfo[i].activeHigh) {
                    if (mLimitSwitchesInfo[i].auxNumber == MoteusAuxNumber::AUX1) {
                        mMoteusAux1Info |= (1 << static_cast<std::size_t>(mLimitSwitchesInfo[i].auxPin));
                    } else if (mLimitSwitchesInfo[i].auxNumber == MoteusAuxNumber::AUX2) {
                        mMoteusAux2Info |= (1 << static_cast<std::size_t>(mLimitSwitchesInfo[i].auxPin));
                    }
                }
            }

            moteus::Controller::Options options;
            moteus::Query::Format queryFormat{};
            queryFormat.aux1_gpio = moteus::kInt8;
            queryFormat.aux2_gpio = moteus::kInt8;
            if (this->isJointDe()) {
                // DE0 and DE1 have absolute encoders
                // They are not used for their internal control loops
                // Instead we request them at the ROS level and send adjust commands periodically
                // Therefore we do not get them as part of normal messages and must request them explicitly
                queryFormat.extra[0] = moteus::Query::ItemFormat{
                        .register_number = moteus::Register::kEncoder1Position,
                        .resolution = moteus::kFloat,
                };
                queryFormat.extra[1] = moteus::Query::ItemFormat{
                        .register_number = moteus::Register::kEncoder1Velocity,
                        .resolution = moteus::kFloat,
                };
            }
            options.query_format = queryFormat;
            mMoteus.emplace(options);
        }

        auto setDesiredThrottle(Percent throttle) -> void {
#ifdef DEBUG_BUILD
            RCLCPP_DEBUG(mNode->get_logger(), "%s throttle set to: %f. Commanding velocity...", mControllerName.c_str(), throttle.rep);
#endif
            setDesiredVelocity(mapThrottleToVelocity(throttle));
        }

        auto setDesiredVelocity(OutputVelocity velocity) -> void {
            // Only check for limit switches if at least one limit switch exists and is enabled
            if (mHasLimit) {
                sendQuery();

                if (auto [isFwdPressed, isBwdPressed] = getPressedLimitSwitchInfo();
                    (velocity > OutputVelocity{0} && isFwdPressed) ||
                    (velocity < OutputVelocity{0} && isBwdPressed)) {
#ifdef DEBUG_BUILD
                    RCLCPP_DEBUG(mNode->get_logger(), "%s hit limit switch. Not commanding velocity", mControllerName.c_str());
#endif
                    setBrake();
                    return;
                }
            }

            velocity = std::clamp(velocity, mMinVelocity, mMaxVelocity);

            if (abs(velocity) < OutputVelocity{1e-5}) {
                setBrake();
            } else {
                moteus::PositionMode::Command command{
                        .position = std::numeric_limits<double>::quiet_NaN(),
                        .velocity = velocity.get(),
                        .maximum_torque = mMaxTorque,
                        .watchdog_timeout = mWatchdogTimeout,
                };
                static constexpr moteus::PositionMode::Format format{
                        .position = moteus::kFloat,
                        .velocity = moteus::kFloat,
                        .maximum_torque = moteus::kInt16,
                        .watchdog_timeout = moteus::kFloat,
                };

                moteus::CanFdFrame velocityFrame = mMoteus->MakePosition(command, &format);
                mDevice.publish_moteus_frame(velocityFrame);
            }

#ifdef DEBUG_BUILD
            RCLCPP_DEBUG(mNode->get_logger(), "Commanding %s velocity to: %f", mControllerName.c_str(), velocity.get());
#endif
        }


        auto setDesiredPosition(OutputPosition position) -> void {
            // Only check for limit switches if at least one limit switch exists and is enabled
            if (mHasLimit) {
                sendQuery();

                if (auto [isFwdPressed, isBwdPressed] = getPressedLimitSwitchInfo();
                    (mCurrentPosition < position && isFwdPressed) ||
                    (mCurrentPosition > position && isBwdPressed)) {
#ifdef DEBUG_BUILD
                    RCLCPP_DEBUG(mNode->get_logger(), "%s hit limit switch. Not commanding position", mControllerName.c_str());
#endif
                    setBrake();
                    return;
                }
            }

            position = std::clamp(position, mMinPosition, mMaxPosition);

            moteus::PositionMode::Command command{
                    .position = position.get(),
                    .velocity = 0.0,
                    .maximum_torque = mMaxTorque,
                    .watchdog_timeout = mWatchdogTimeout,
            };
            static constexpr moteus::PositionMode::Format format{
                    .position = moteus::kFloat,
                    .velocity = moteus::kFloat,
                    .maximum_torque = moteus::kInt16,
                    .watchdog_timeout = moteus::kFloat,
            };

            moteus::CanFdFrame positionFrame = mMoteus->MakePosition(command, &format);
            mDevice.publish_moteus_frame(positionFrame);

#ifdef DEBUG_BUILD
            RCLCPP_DEBUG(mNode->get_logger(), "Commanding %s position to: %f", mControllerName.c_str(), position.get());
#endif
        }

        auto processCANMessage(msg::CAN::ConstSharedPtr const& msg) -> void {
            assert(msg->source == mControllerName);
            assert(msg->destination == mMasterName);
            auto result = moteus::Query::Parse(msg->data.data(), msg->data.size());

            if (this->isJointDe()) {
                mCurrentPosition = OutputPosition{result.extra[0].value}; // Get value of absolute encoder if its joint_de0/1
                mCurrentVelocity = OutputVelocity{result.extra[1].value};
            } else {
                mCurrentPosition = OutputPosition{result.position};
                mCurrentVelocity = OutputVelocity{result.velocity};
            }
            mCurrentEffort = result.torque;

            mErrorState = moteusErrorCodeToErrorState(result.mode, static_cast<ErrorCode>(result.fault));
            mState = moteusModeToState(result.mode);

            mMoteusAux1Info = result.aux1_gpio;
            mMoteusAux2Info = result.aux2_gpio;

            if (result.mode == moteus::Mode::kPositionTimeout || result.mode == moteus::Mode::kFault) {
                setStop();
                RCLCPP_WARN(mNode->get_logger(), "Position timeout hit");
            }
        }

        auto setStop() -> void {
            moteus::CanFdFrame setStopFrame = mMoteus->MakeStop();
            mDevice.publish_moteus_frame(setStopFrame);
        }

        auto setBrake() -> void {
            moteus::CanFdFrame setBrakeFrame = mMoteus->MakeBrake();
            mDevice.publish_moteus_frame(setBrakeFrame);
        }

        auto getPressedLimitSwitchInfo() -> MoteusLimitSwitchInfo {
            MoteusLimitSwitchInfo result{};

            for (std::size_t i = 0; i < MAX_NUM_LIMIT_SWITCHES; ++i) {
                if (mLimitSwitchesInfo[i].present && mLimitSwitchesInfo[i].enabled) {
                    std::uint8_t auxInfo = 0;
                    if (mLimitSwitchesInfo[i].auxNumber == MoteusAuxNumber::AUX1) {
                        auxInfo = mMoteusAux1Info;
                    } else if (mLimitSwitchesInfo[i].auxNumber == MoteusAuxNumber::AUX2) {
                        auxInfo = mMoteusAux2Info;
                    }
                    bool gpioState = auxInfo & (1 << static_cast<std::size_t>(mLimitSwitchesInfo[i].auxPin));
                    mLimitHit[i] = gpioState == mLimitSwitchesInfo[i].activeHigh;
                }
                result.isForwardPressed = (mLimitHit[i] && mLimitSwitchesInfo[i].limitsForward) || result.isForwardPressed;
                result.isBackwardPressed = (mLimitHit[i] && !mLimitSwitchesInfo[i].limitsForward) || result.isBackwardPressed;
                if (mLimitSwitchesInfo[i].usedForReadjustment && mLimitHit[i]) {
                    adjust(mLimitSwitchesInfo[i].readjustPosition);
                }
            }

            return result;
        }

        auto adjust(OutputPosition position) -> void {
            position = std::clamp(position, mMinPosition, mMaxPosition);
            moteus::OutputExact::Command command{
                    .position = position.get(),
            };
            moteus::OutputExact::Command outputExactCmd{command};
            moteus::CanFdFrame setPositionFrame = mMoteus->MakeOutputExact(outputExactCmd);
            mDevice.publish_moteus_frame(setPositionFrame);
        }

        auto sendQuery() -> void {
            moteus::CanFdFrame queryFrame = mMoteus->MakeQuery();
            mDevice.publish_moteus_frame(queryFrame);
        }

    private:
        [[nodiscard]] auto mapThrottleToVelocity(Percent throttle) const -> OutputVelocity {
            throttle = std::clamp(throttle, -1_percent, 1_percent);
            return abs(throttle) * (throttle > 0_percent ? mMaxVelocity : mMinVelocity);
        }

        // Converts moteus error codes and mode codes to std::string descriptions
        static auto moteusErrorCodeToErrorState(moteus::Mode motor_mode, ErrorCode motor_error_code) -> std::string {
            if (motor_mode != moteus::Mode::kFault) return "No Error";
            switch (motor_error_code) {
                case ErrorCode::DmaStreamTransferError:
                    return "DMA Stream Transfer Error";
                case ErrorCode::DmaStreamFifiError:
                    return "DMA Stream FIFO Error";
                case ErrorCode::UartOverrunError:
                    return "UART Overrun Error";
                case ErrorCode::UartFramingError:
                    return "UART Framing Error";
                case ErrorCode::UartNoiseError:
                    return "UART Noise Error";
                case ErrorCode::UartBufferOverrunError:
                    return "UART Buffer Overrun Error";
                case ErrorCode::UartParityError:
                    return "UART Parity Error";
                case ErrorCode::CalibrationFault:
                    return "Calibration Fault";
                case ErrorCode::MotorDriverFault:
                    return "Motor Driver Fault";
                case ErrorCode::OverVoltage:
                    return "Over Voltage";
                case ErrorCode::EncoderFault:
                    return "Encoder Fault";
                case ErrorCode::MotorNotConfigured:
                    return "Motor Not Configured";
                case ErrorCode::PwmCycleOverrun:
                    return "PWM Cycle Overrun";
                case ErrorCode::OverTemperature:
                    return "Over Temperature";
                case ErrorCode::StartOutsideLimit:
                    return "Start Outside Limit";
                case ErrorCode::UnderVoltage:
                    return "Under Voltage";
                case ErrorCode::ConfigChanged:
                    return "Configuration Changed";
                case ErrorCode::ThetaInvalid:
                    return "Theta Invalid";
                case ErrorCode::PositionInvalid:
                    return "Position Invalid";
                default:
                    return "Unknown Error";
            }
        }

        static auto moteusModeToState(moteus::Mode motor_mode) -> std::string {
            switch (motor_mode) {
                case moteus::Mode::kStopped:
                    return "Motor Stopped";
                case moteus::Mode::kFault:
                    return "Motor Fault";
                case moteus::Mode::kEnabling:
                    return "Motor Enabling";
                case moteus::Mode::kCalibrating:
                    return "Motor Calibrating";
                case moteus::Mode::kCalibrationComplete:
                    return "Motor Calibration Complete";
                case moteus::Mode::kPwm:
                    return "Motor Pwm";
                case moteus::Mode::kVoltage:
                    return "Voltage Operating Mode";
                case moteus::Mode::kVoltageFoc:
                    return "Voltage FOC Operating Mode";
                case moteus::Mode::kVoltageDq:
                    return "Voltage DQ Operating Mode";
                case moteus::Mode::kCurrent:
                    return "Current Operating Mode";
                case moteus::Mode::kPosition:
                    return "Position Operating Mode";
                case moteus::Mode::kPositionTimeout:
                    return "Position Timeout";
                case moteus::Mode::kZeroVelocity:
                    return "Zero Velocity";
                case moteus::Mode::kStayWithin:
                    return "Motor Stay Within";
                case moteus::Mode::kMeasureInd:
                    return "Measure Ind";
                case moteus::Mode::kBrake:
                    return "Motor Brake";
                default:
                    return "Unknown Mode";
            }
        }
    };

} // namespace mrover
