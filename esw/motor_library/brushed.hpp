#pragma once

#include "controller.hpp"

namespace mrover {

    template<IsUnit TOutputPosition>
    class BrushedController final : public ControllerBase<BrushedController<TOutputPosition>> {
    public:
        using OutputPosition = TOutputPosition;
        using OutputVelocity = compound_unit<OutputPosition, inverse<Seconds>>;

    private:
        using Base = ControllerBase<BrushedController>;

        using Base::mControllerName;
        using Base::mCurrent;
        using Base::mDevice;
        using Base::mErrorState;
        using Base::mLimitHit;
        using Base::mMasterName;
        using Base::mNode;
        using Base::mPosition;
        using Base::mState;
        using Base::mVelocity;

        enum struct Mode_t : uint8_t {
            STOPPED = 0,
            FAULT = 1,
            THROTTLE = 5,
            POSITION = 6,
            VELOCITY = 7,
        };

        enum struct BMCError_t : uint8_t {
            NONE,                           // no error
            NO_MODE,                        // no mode selected
            INVALID_CONFIGURATION_FOR_MODE, // in position or velocity mode without feedback mechanism configured to close the loop
            INVALID_FLASH_CONFIG,           // signal that there are too many encoders connected or something
            WWDG_EXPIRED,                   // watchdog expired
            UNCALIBRATED,                   // received position or velocity mode before calibrated
            CAN_ERROR_FATAL,                // unrecoverable CAN error encountered
            I2C_ERROR_FATAL,                // unrecoverable I2C error encountered
            SPI_ERROR_FATAL,                // unrecoverable SPI error encountered
        };

        static auto byteToMode(uint8_t const value) -> Mode_t {
            switch (value) {
                case 0:
                    return Mode_t::STOPPED;
                case 1:
                    return Mode_t::FAULT;
                case 5:
                    return Mode_t::THROTTLE;
                case 6:
                    return Mode_t::POSITION;
                case 7:
                    return Mode_t::VELOCITY;
                default:
                    throw std::invalid_argument("invalid mode");
            }
        }

        static auto errorToString(BMCError_t const error) -> std::string {
            switch (error) {
                case BMCError_t::NONE:
                    return "None";
                case BMCError_t::NO_MODE:
                    return "No mode selected";
                case BMCError_t::INVALID_CONFIGURATION_FOR_MODE:
                    return "Invalid configuration for mode";
                case BMCError_t::INVALID_FLASH_CONFIG:
                    return "Invalid flash configuration";
                case BMCError_t::WWDG_EXPIRED:
                    return "CAN watchdog expired";
                case BMCError_t::UNCALIBRATED:
                    return "Uncalibrated";
                case BMCError_t::CAN_ERROR_FATAL:
                    return "Fatal CAN error";
                case BMCError_t::I2C_ERROR_FATAL:
                    return "Fatal I2C error";
                case BMCError_t::SPI_ERROR_FATAL:
                    return "Fatal SPI error";
                default:
                    return "Unknown Error";
            }
        }

        Mode_t mReportedMode{Mode_t::STOPPED};

        void ensureMode(Mode_t const targetMode) {
            if (mReportedMode != targetMode) {
                mDevice.publishMessage(BMCModeCmd{static_cast<uint8_t>(targetMode), 1});
            }
        }

    public:
        BrushedController(rclcpp::Node::SharedPtr node, std::string masterName, std::string controllerName)
            : Base{std::move(node), std::move(masterName), std::move(controllerName)} {
        }

        auto setDesiredThrottle(Percent const throttle) -> void {
            ensureMode(Mode_t::THROTTLE);
            mDevice.publishMessage(BMCTargetCmd{throttle.get(), 1});
        }

        auto setDesiredPosition(OutputPosition const position) -> void {
            ensureMode(Mode_t::POSITION);
            mDevice.publishMessage(BMCTargetCmd{position.get(), 1});
        }

        auto setDesiredVelocity(OutputVelocity const velocity) -> void {
            ensureMode(Mode_t::VELOCITY);
            mDevice.publishMessage(BMCTargetCmd{velocity.get(), 1});
        }

        void processMessage(CANMsg_t const& msg) {
            std::visit([this](auto const& decoded) -> auto {
                using T = std::decay_t<decltype(decoded)>;

                if constexpr (std::is_same_v<T, BMCMotorState>) {
                    this->mPosition = decoded.position;
                    this->mVelocity = decoded.velocity;
                    this->mCurrent = decoded.current;
                    this->mLimitHit[0] = static_cast<bool>(decoded.limit_a);
                    this->mLimitHit[1] = static_cast<bool>(decoded.limit_b);

                    // update internal bmc state
                    auto mode = byteToMode(decoded.mode);
                    this->mReportedMode = mode;
                    this->mState = (mode == Mode_t::FAULT) ? "Fault" : "Running";
                    this->mErrorState = errorToString(static_cast<BMCError_t>(decoded.fault_code));
                } else if constexpr (std::is_same_v<T, BMCAck>) {
                    RCLCPP_INFO(mNode->get_logger(), "BMC Ack received: %u", decoded.data);
                }
            },
                       msg);
        }
    };
} // namespace mrover
