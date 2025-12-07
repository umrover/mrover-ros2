#include "mrover/msg/detail/dynamixel_set_position__struct.hpp"
#include "mrover/srv/detail/dynamixel_get_position__struct.hpp"
#include "mrover/srv/detail/dynamixel_get_current__struct.hpp"
#include <cstdlib>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_sdk/port_handler.h>
#include <dynamixel_sdk/packet_handler.h>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/cmdline_parser.h>
#include <string>

#include <mrover/msg/dynamixel_set_position.hpp>
#include <mrover/srv/dynamixel_get_position.hpp>


class DynamixelServoNode : public rclcpp::Node
{
public:
  using SetPosition = mrover::msg::DynamixelSetPosition;
  using GetPosition = mrover::srv::DynamixelGetPosition;
  using GetCurrent = mrover::srv::DynamixelGetCurrent;

  DynamixelServoNode();
  virtual ~DynamixelServoNode();

  enum DirectionState {
    DirectionOptimal,
    DirectionCounterClockwise,
    DirectionClockwise
  };

private:
  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;

  int present_position;

  rclcpp::Service<GetCurrent>::SharedPtr get_current_server_;
};

#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_CURRENT 126

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

DynamixelServoNode::DynamixelServoNode()
: Node("read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Run dynamixel servo node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  set_position_subscriber_ =
  this->create_subscription<SetPosition>(
  "set_position",
  QOS_RKL10V,
  [this](const SetPosition::SharedPtr msg) -> void
  {
    uint8_t dxl_error = 0;

    // Position Value of X series is 4 byte data.
    // Convert angle to position value (0-4095 corresponds to 0-360 degrees)
    uint32_t target_position = static_cast<uint32_t>((static_cast<float>(msg->position) / 360.0f) * 4096.0f) % 4096;

    uint32_t present_position;
    dxl_comm_result = packetHandler->read4ByteTxRx(
      portHandler,
      static_cast<uint8_t>(msg->id),
      ADDR_PRESENT_POSITION,
      reinterpret_cast<uint32_t *>(&present_position),
      &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read present position for ID: %d", msg->id);
      return;
    }

    uint32_t goal_position;
    // Get the actual current position modulo 4096
    uint32_t current_position = present_position % 4096;
    
    // Calculate the signed difference (accounting for overflow)
    int32_t raw_diff = static_cast<int32_t>(target_position) - static_cast<int32_t>(current_position);
    
    // Normalize the difference to the range [-2048, 2047]
    int32_t normalized_diff = raw_diff;
    switch (msg->direction_state) {

      case DirectionOptimal:
        if (normalized_diff > 2048) {
          normalized_diff -= 4096;  // Go the other (shorter) way around
        } else if (normalized_diff < -2048) {
          normalized_diff += 4096;  // Go the other (shorter) way around
        }
        break;
      case DirectionClockwise: // clockwise
        if (normalized_diff < 0)
        {
          normalized_diff += 4096;
        }
        break;
      case DirectionCounterClockwise: // counter clockwise
        if (normalized_diff > 0)
        {
          normalized_diff -= 4096;
        }
        break;
    }
    
    // Calculate the optimal goal position
    if (normalized_diff >= 0) {
      goal_position = present_position + normalized_diff;
    } else {
      goal_position = present_position - static_cast<uint32_t>(-normalized_diff);
    }

    // Write Goal Position
    dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler,
      static_cast<uint8_t>(msg->id),
      ADDR_GOAL_POSITION,
      goal_position,
      &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
      RCLCPP_ERROR(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
    } else {
      RCLCPP_INFO(this->get_logger(), 
                  "Set [ID: %d] [Current: %d°/%d] [Target: %d°/%d] [Goal: %d] [Diff: %d]", 
                  msg->id,
                  static_cast<int>((current_position * 360.0f) / 4096.0f),
                  current_position,
                  msg->position,
                  target_position,
                  goal_position,
                  normalized_diff);
    }
  }
  );

  auto get_present_position =
    [this](
    const std::shared_ptr<GetPosition::Request> request,
    std::shared_ptr<GetPosition::Response> response) -> void
    {
      // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position),
        &dxl_error
      );

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Position: %d]",
        request->id,
        present_position
      );

      response->position = present_position;
    };

  get_position_server_ = create_service<GetPosition>("get_position", get_present_position);
}

DynamixelServoNode::~DynamixelServoNode()
{
}

void setupDynamixel(uint8_t dxl_id)
{
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    4,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("Dynamixel_Servo_Node"), "Failed to set Position Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("Dynamixel_Servo_Node"), "Succeeded to set Position Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("Dynamixel_Servo_Node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("Dynamixel_Servo_Node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("Dynamixel_Servo_Node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("Dynamixel_Servo_Node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("Dynamixel_Servo_Node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("Dynamixel_Servo_Node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<DynamixelServoNode>();
  rclcpp::spin(readwritenode);
  rclcpp::shutdown();

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    0,
    &dxl_error
  );

  return 0;
}