#pragma once

#include <dynamixel_sdk/packet_handler.h>
#include <dynamixel_sdk/port_handler.h>
#include <mutex>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <unordered_set>

namespace mrover {

    static constexpr float PROTOCOL_VERSION = 2.0;
    static constexpr uint32_t SERVO_BAUDRATE = 57600;

    class U2D2 {
        U2D2() : mPortHandler{nullptr}, mPacketHandler{nullptr}, mInitialized{false} {}

        dynamixel::PortHandler* mPortHandler;
        dynamixel::PacketHandler* mPacketHandler;
        std::unordered_set<uint8_t> mServos;
        mutable std::mutex mBusMutex;
        bool mInitialized;

    public:
        enum class Status : int32_t {
            Active = 400,
            HardwareFailure = 401,
            Success = 0,
            FailedToOpenPort = 1,
            FailedToSetBaud = 2,
            CommPortBusy = -1000,
            CommTxFail = -1001,
            CommRxFail = -1002,
            CommTxError = -2000,
            CommRxWaiting = -3000,
            CommRxTimeout = -3001,
            CommRxCorrupt = -3002,
            CommNotAvailable = -9000,
        };


        // delete copy ctor and assignment operator
        U2D2(U2D2 const&) = delete;
        auto operator=(U2D2 const&) -> U2D2& = delete;

        static auto getInstance() -> U2D2* {
            static U2D2 inst;
            return &inst;
        }

        static auto getSharedInstance() -> std::shared_ptr<U2D2> {
            static std::shared_ptr<U2D2> inst{getInstance(), [](U2D2*) {}};
            return inst;
        }

        auto init(std::string const& deviceName) -> Status {
            if (mInitialized) return Status::Success;

            mPortHandler = dynamixel::PortHandler::getPortHandler(deviceName.c_str());
            mPacketHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

            if (!mPortHandler->openPort()) return Status::FailedToOpenPort;
            if (!mPortHandler->setBaudRate(SERVO_BAUDRATE)) return Status::FailedToSetBaud;

            mInitialized = true;
            return Status::Success;
        }

        auto registerServo(uint8_t const id) -> void {
            if (mServos.contains(id)) {
                RCLCPP_FATAL(rclcpp::get_logger("u2d2"), "duplicate servo id registered");
                rclcpp::shutdown();
            }
            mServos.emplace(id);
        }

        auto write1Byte(int const addr, uint8_t const data, uint8_t const id, uint8_t* hardwareStatus) const -> Status {
            std::lock_guard<std::mutex> lock(mBusMutex);
            return static_cast<Status>(mPacketHandler->write1ByteTxRx(mPortHandler, id, addr, data, hardwareStatus));
        }

        auto write2Byte(int const addr, uint16_t const data, uint8_t const id, uint8_t* hardwareStatus) const -> Status {
            std::lock_guard<std::mutex> lock(mBusMutex);
            return static_cast<Status>(mPacketHandler->write2ByteTxRx(mPortHandler, id, addr, data, hardwareStatus));
        }

        auto write4Byte(int const addr, uint32_t const data, uint8_t const id, uint8_t* hardwareStatus) const -> Status {
            std::lock_guard<std::mutex> lock(mBusMutex);
            return static_cast<Status>(mPacketHandler->write4ByteTxRx(mPortHandler, id, addr, data, hardwareStatus));
        }

        auto read1Byte(int const addr, uint8_t& data, uint8_t const id, uint8_t* hardwareStatus) const -> Status {
            std::lock_guard<std::mutex> lock(mBusMutex);
            return static_cast<Status>(mPacketHandler->read1ByteTxRx(mPortHandler, id, addr, &data, hardwareStatus));
        }

        auto read2Byte(int const addr, uint16_t& data, uint8_t const id, uint8_t* hardwareStatus) const -> Status {
            std::lock_guard<std::mutex> lock(mBusMutex);
            return static_cast<Status>(mPacketHandler->read2ByteTxRx(mPortHandler, id, addr, &data, hardwareStatus));
        }

        auto read4Byte(int const addr, uint32_t& data, uint8_t const id, uint8_t* hardwareStatus) const -> Status {
            std::lock_guard<std::mutex> lock(mBusMutex);
            return static_cast<Status>(mPacketHandler->read4ByteTxRx(mPortHandler, id, addr, &data, hardwareStatus));
        }

        static auto stringifyStatus(Status const status) -> std::string {
            switch (status) {
                case Status::Active:
                    return "Active";
                case Status::HardwareFailure:
                    return "HardwareFailure";
                case Status::Success:
                    return "Success";
                case Status::FailedToOpenPort:
                    return "FailedToOpenPort";
                case Status::FailedToSetBaud:
                    return "FailedToSetBaud";
                case Status::CommPortBusy:
                    return "CommPortBusy";
                case Status::CommTxFail:
                    return "CommTxFail";
                case Status::CommRxFail:
                    return "CommRxFail";
                case Status::CommTxError:
                    return "CommTxError";
                case Status::CommRxWaiting:
                    return "CommRxWaiting";
                case Status::CommRxTimeout:
                    return "CommRxTimeout";
                case Status::CommRxCorrupt:
                    return "CommRxCorrupt";
                case Status::CommNotAvailable:
                    return "CommNotAvailable";
                default:
                    return "UnknownStatus";
            }
        }
    };

} // namespace mrover
