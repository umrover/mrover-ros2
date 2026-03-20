#pragma once

#include <cstdint>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_set>

// Protocol version and constants
#define PROTOCOL_VERSION 2.0
#define SERVO_BAUDRATE 57600

namespace mrover {

    class u2d2 {
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

        static auto registerServo(uint8_t id) -> void;
        static auto stringifyStatus(Status status) -> std::string;
        static auto init(std::string const& deviceName) -> Status;

        // Write operations
        static auto write1Byte(int addr, uint8_t data, uint8_t id, uint8_t* hardwareStatus) -> Status;
        static auto write2Byte(int addr, uint16_t data, uint8_t id, uint8_t* hardwareStatus) -> Status;
        static auto write4Byte(int addr, uint32_t data, uint8_t id, uint8_t* hardwareStatus) -> Status;

        // Read operations
        static auto read1Byte(int addr, uint8_t& data, uint8_t id, uint8_t* hardwareStatus) -> Status;
        static auto read2Byte(int addr, uint16_t& data, uint8_t id, uint8_t* hardwareStatus) -> Status;
        static auto read4Byte(int addr, uint32_t& data, uint8_t id, uint8_t* hardwareStatus) -> Status;

    private:
        static dynamixel::PortHandler* portHandler;
        static dynamixel::PacketHandler* packetHandler;
        static std::unordered_set<uint8_t> servos;
    };

} // namespace mrover