#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <moteus/moteus.h>
#include <moteus/moteus_multiplex.h>

#include <can_device.hpp>
#include <controller.hpp>

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

    public:
        struct Config {
            OutputVelocity minVelocity = OutputVelocity{-1.0};
            OutputVelocity maxVelocity = OutputVelocity{1.0};
            OutputPosition minPosition = OutputPosition{-1.0};
            OutputPosition maxPosition = OutputPosition{1.0};
            double maxTorque = 0.3;
            double watchdogTimeout = 0.25;

            bool limitSwitch0Present = false;
            bool limitSwitch0Enabled = true;
            bool limitSwitch0LimitsFwd = false;
            bool limitSwitch0ActiveHigh = true;
            bool limitSwitch0UsedForReadjustment = false;
            OutputPosition limitSwitch0ReadjustPosition = OutputPosition{0.0};
            bool limitSwitch1Present = false;
            bool limitSwitch1Enabled = true;
            bool limitSwitch1LimitsFwd = false;
            bool limitSwitch1ActiveHigh = true;
            bool limitSwitch1UsedForReadjustment = false;
            OutputPosition limitSwitch1ReadjustPosition = OutputPosition{0.0};
        };

        BrushlessController(rclcpp::Node::SharedPtr node, std::string masterName, std::string controllerName, Config config)
            : Base{std::move(node), std::move(masterName), std::move(controllerName)}, mConfig{config} {

            // if active low, we want to make the default value make it believe that
            // the limit switch is NOT pressed.
            // This is because we may not receive the newest query message from the moteus
            // as a result of either testing or startup.
            // TODO: make configurable
            if (mConfig.limitSwitch0Present && !mConfig.limitSwitch0ActiveHigh) {
                mMoteusAux1Info |= 0b001;
            }
            if (mConfig.limitSwitch1Present && !mConfig.limitSwitch1ActiveHigh) {
                mMoteusAux1Info |= 0b100;
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
            if ((mConfig.limitSwitch0Enabled && mConfig.limitSwitch0Present) || (mConfig.limitSwitch1Enabled && mConfig.limitSwitch0Present)) {
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

            velocity = std::clamp(velocity, mConfig.minVelocity, mConfig.maxVelocity);

            if (abs(velocity) < OutputVelocity{1e-5}) {
                setBrake();
            } else {
                moteus::PositionMode::Command command{
                        .position = std::numeric_limits<double>::quiet_NaN(),
                        .velocity = velocity.get(),
                        .maximum_torque = mConfig.maxTorque,
                        .watchdog_timeout = mConfig.watchdogTimeout,
                };

                moteus::CanFdFrame positionFrame = mMoteus->MakePosition(command);
                mDevice.publish_moteus_frame(positionFrame);
            }

#ifdef DEBUG_BUILD
            RCLCPP_DEBUG(mNode->get_logger(), "Commanding %s velocity to: %f", mControllerName.c_str(), velocity.get());
#endif
        }


        auto setDesiredPosition(OutputPosition position) -> void {
            // Only check for limit switches if at least one limit switch exists and is enabled
            if ((mConfig.limitSwitch0Enabled && mConfig.limitSwitch0Present) || (mConfig.limitSwitch1Enabled && mConfig.limitSwitch0Present)) {
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

            position = std::clamp(position, mConfig.minPosition, mConfig.maxPosition);

            moteus::PositionMode::Command command{
                    .position = position.get(),
                    .velocity = 0.0,
                    .maximum_torque = mConfig.maxTorque,
                    .watchdog_timeout = mConfig.watchdogTimeout,
            };
            moteus::CanFdFrame positionFrame = mMoteus->MakePosition(command);
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
            if (mConfig.limitSwitch0Present && mConfig.limitSwitch0Enabled) {
                // TODO: make configurable
                bool gpioState = 0b001 & mMoteusAux1Info;
                mLimitHit[0] = gpioState == mConfig.limitSwitch0ActiveHigh;
            }
            if (mConfig.limitSwitch1Present && mConfig.limitSwitch1Enabled) {
                bool gpioState = 0b100 & mMoteusAux1Info;
                mLimitHit[1] = gpioState == mConfig.limitSwitch1ActiveHigh;
            }

            MoteusLimitSwitchInfo result{
                    .isForwardPressed = (mLimitHit[0] && mConfig.limitSwitch0LimitsFwd) || (mLimitHit[1] && mConfig.limitSwitch1LimitsFwd),
                    .isBackwardPressed = (mLimitHit[0] && !mConfig.limitSwitch0LimitsFwd) || (mLimitHit[1] && !mConfig.limitSwitch1LimitsFwd),
            };

            if (result.isForwardPressed) {
                adjust(mConfig.limitSwitch0ReadjustPosition);
            } else if (result.isBackwardPressed) {
                adjust(mConfig.limitSwitch1ReadjustPosition);
            }

            return result;
        }

        auto adjust(OutputPosition position) -> void {
            position = std::clamp(position, mConfig.minPosition, mConfig.maxPosition);
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
        std::optional<moteus::Controller> mMoteus;
        Config mConfig;
        std::int8_t mMoteusAux1Info{}, mMoteusAux2Info{};

        [[nodiscard]] auto mapThrottleToVelocity(Percent throttle) const -> OutputVelocity {
            throttle = std::clamp(throttle, -1_percent, 1_percent);
            return abs(throttle) * (throttle > 0_percent ? mConfig.maxVelocity : mConfig.minVelocity);
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
