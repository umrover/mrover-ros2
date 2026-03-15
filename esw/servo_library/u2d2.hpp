#include <cstdint>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_sdk/packet_handler.h>
#include <dynamixel_sdk/port_handler.h>
#include <string>
#include <unordered_set>
#include <rclcpp/rclcpp.hpp>

// Protocol version
#define PROTOCOL_VERSION 2.0 // Default Protocol version of DYNAMIXEL X series.
#define SERVO_BAUDRATE 57600 // Baud rate of dynamixel devices

class u2d2
{
public:
  enum class U2D2Status {
      Active = 400,
      HardwareFailure = 401,
      Success = 0,
      FailedToOpenPort,
      FailedToSetBaud,
      CommPortBusy = -1000,
      CommTxFail = -1001,
      CommRxFail = -1002,
      CommTxError = -2000,
      CommRxWaiting = -3000,
      CommRxTimeout = -3001,
      CommRxCorrupt = -3002,
      CommNotAvailable = -9000,
  };

  static auto registerServo(uint8_t id)
  {
    if (servos.count(id) != 0)
    {
      auto logger = rclcpp::get_logger("u2d2_logger");
      RCLCPP_FATAL(logger, "A critical error occurred in the helper function!");
      
      // Trigger a global shutdown
      rclcpp::shutdown();
    }
    servos.emplace(id);
  }

  static auto stringifyStatus(U2D2Status status) -> std::string
  {
      switch (status) {
          case U2D2Status::Active:
              return "Active";
          case U2D2Status::HardwareFailure:
              return "HardwareFailure";
          case U2D2Status::Success:
              return "Success";
          case U2D2Status::FailedToOpenPort:
              return "FailedToOpenPort";
          case U2D2Status::FailedToSetBaud:
              return "FailedToSetBaud";
          case U2D2Status::CommPortBusy:
              return "CommPortBusy";
          case U2D2Status::CommTxFail:
              return "CommTxFail";
          case U2D2Status::CommRxFail:
              return "CommRxFail";
          case U2D2Status::CommTxError:
              return "CommTxError";
          case U2D2Status::CommRxWaiting:
              return "CommRxWaiting";
          case U2D2Status::CommRxTimeout:
              return "CommRxTimeout";
          case U2D2Status::CommRxCorrupt:
              return "CommRxCorrupt";
          case U2D2Status::CommNotAvailable:
              return "CommNotAvailable";
          default:
              return "UnknownStatus";
      }
  }

  static auto init(std::string const& deviceName) -> U2D2Status {
      portHandler = dynamixel::PortHandler::getPortHandler(deviceName.c_str());
      packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

      int dxlCommResult;

      // Open Serial Port
      dxlCommResult = portHandler->openPort();
      if (!dxlCommResult) {
          return U2D2Status::FailedToOpenPort;
      }

      // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
      dxlCommResult = portHandler->setBaudRate(SERVO_BAUDRATE);
      if (!dxlCommResult) {
          return U2D2Status::FailedToSetBaud;
      }

      return U2D2Status::Success;
  }

  static auto write1Byte(int addr, uint8_t data, uint8_t id, uint8_t* hardwareStatus) -> U2D2Status {
      return static_cast<U2D2Status>(packetHandler->write1ByteTxRx(
              portHandler,
              id,
              addr,
              data,
              hardwareStatus));
  }

  static auto write2Byte(int addr, uint16_t data, uint8_t id, uint8_t* hardwareStatus) -> U2D2Status {
      return static_cast<U2D2Status>(packetHandler->write2ByteTxRx(
              portHandler,
              id,
              addr,
              data,
              hardwareStatus));
  }

  static auto write4Byte(int addr, uint32_t data, uint8_t id, uint8_t* hardwareStatus) -> U2D2Status {
      return static_cast<U2D2Status>(packetHandler->write4ByteTxRx(
              portHandler,
              id,
              addr,
              data,
              hardwareStatus));
  }


  static auto read1Byte(int addr, uint8_t& data, uint8_t id, uint8_t* hardwareStatus) -> U2D2Status {
      return static_cast<U2D2Status>(packetHandler->read1ByteTxRx(
              portHandler,
              id,
              addr,
              &data,
              hardwareStatus));
  }

  static auto read2Byte(int addr, uint16_t& data, uint8_t id, uint8_t* hardwareStatus) -> U2D2Status {
      return static_cast<U2D2Status>(packetHandler->read2ByteTxRx(
              portHandler,
              id,
              addr,
              &data,
              hardwareStatus));
  }

  static auto read4Byte(int addr, uint32_t& data, uint8_t id, uint8_t* hardwareStatus) -> U2D2Status {
      return static_cast<U2D2Status>(packetHandler->read4ByteTxRx(
              portHandler,
              id,
              addr,
              &data,
              hardwareStatus));
  }

  inline static dynamixel::PortHandler* portHandler;
  inline static dynamixel::PacketHandler* packetHandler;
  inline static std::unordered_set<uint8_t> servos;
};