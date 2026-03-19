#include "u2d2.hpp"

using namespace mrover;

// Static member definitions
dynamixel::PortHandler* u2d2::portHandler = nullptr;
dynamixel::PacketHandler* u2d2::packetHandler = nullptr;
std::unordered_set<uint8_t> u2d2::servos;

void u2d2::registerServo(uint8_t id) {
    if (servos.find(id) != servos.end()) {
        auto logger = rclcpp::get_logger("u2d2_logger");
        RCLCPP_FATAL(logger, "Duplicate registration attempt for Servo ID: %d", id);
        rclcpp::shutdown();
    }
    servos.emplace(id);
}

auto u2d2::stringifyStatus(Status status) -> std::string {
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

auto u2d2::init(std::string const& deviceName) -> u2d2::Status {
    portHandler = dynamixel::PortHandler::getPortHandler(deviceName.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort()) {
        return Status::FailedToOpenPort;
    }

    if (!portHandler->setBaudRate(SERVO_BAUDRATE)) {
        return Status::FailedToSetBaud;
    }

    return Status::Success;
}

// --- Write Implementations ---

auto u2d2::write1Byte(int addr, uint8_t data, uint8_t id, uint8_t* hardwareStatus) -> u2d2::Status {
    return static_cast<Status>(packetHandler->write1ByteTxRx(portHandler, id, addr, data, hardwareStatus));
}

auto u2d2::write2Byte(int addr, uint16_t data, uint8_t id, uint8_t* hardwareStatus) -> u2d2::Status {
    return static_cast<Status>(packetHandler->write2ByteTxRx(portHandler, id, addr, data, hardwareStatus));
}

auto u2d2::write4Byte(int addr, uint32_t data, uint8_t id, uint8_t* hardwareStatus) -> u2d2::Status {
    return static_cast<Status>(packetHandler->write4ByteTxRx(portHandler, id, addr, data, hardwareStatus));
}

// --- Read Implementations ---

auto u2d2::read1Byte(int addr, uint8_t& data, uint8_t id, uint8_t* hardwareStatus) -> u2d2::Status {
    return static_cast<Status>(packetHandler->read1ByteTxRx(portHandler, id, addr, &data, hardwareStatus));
}

auto u2d2::read2Byte(int addr, uint16_t& data, uint8_t id, uint8_t* hardwareStatus) -> u2d2::Status {
    return static_cast<Status>(packetHandler->read2ByteTxRx(portHandler, id, addr, &data, hardwareStatus));
}

auto u2d2::read4Byte(int addr, uint32_t& data, uint8_t id, uint8_t* hardwareStatus) -> u2d2::Status {
    return static_cast<Status>(packetHandler->read4ByteTxRx(portHandler, id, addr, &data, hardwareStatus));
}