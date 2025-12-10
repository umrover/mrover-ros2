#include "mrover/msg/detail/dynamixel_set_position__struct.hpp"
#include "mrover/msg/detail/dynamixel_set_current_limit__struct.hpp"
#include "mrover/srv/detail/dynamixel_get_position__struct.hpp"
#include "mrover/srv/detail/dynamixel_get_current__struct.hpp"
#include <cstdint>
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
#include <mrover/srv/dynamixel_get_current.hpp>
#include <mrover/msg/dynamixel_set_current_limit.hpp>
#include <unordered_map>


// Default setting
#define DEFAULT_BAUDRATE 57600
#define DEFAULT_DEVICE_NAME "/dev/ttyUSB0"
#define DEFAULT_CURRENT_LIMIT 1750
#define DEFAULT_POSITION_P_GAIN 400
#define DEFAULT_POSITION_I_GAIN 0
#define DEFAULT_POSITION_D_GAIN 0
#define DEFAULT_VELOCITY_P_GAIN 180
#define DEFAULT_VELOCITY_I_GAIN 0

class DynamixelServoNode : public rclcpp::Node
{
public:
  using SetPosition = mrover::msg::DynamixelSetPosition;
  using SetCurrentLimit  = mrover::msg::DynamixelSetCurrentLimit;
  using GetPosition = mrover::srv::DynamixelGetPosition;
  using GetCurrent = mrover::srv::DynamixelGetCurrent;
  using dynamixel_id_t = uint8_t;

  struct DynamixelServo
  {
    uint32_t positionPGain = DEFAULT_POSITION_P_GAIN;
    uint32_t positionIGain = DEFAULT_POSITION_I_GAIN;
    uint32_t positionDGain = DEFAULT_POSITION_D_GAIN;
    uint32_t velocityPGain = DEFAULT_VELOCITY_P_GAIN;
    uint32_t velocityIGain = DEFAULT_VELOCITY_I_GAIN;
    uint32_t currentLimit = DEFAULT_CURRENT_LIMIT;


    DynamixelServo(const DynamixelServo& state) = default;
    DynamixelServo() = default;
  };

  struct DynamixelState
  {

    const int baudRate = DEFAULT_BAUDRATE;
    const std::string deviceName = DEFAULT_DEVICE_NAME;
    std::unordered_map<dynamixel_id_t, DynamixelServo> servos = {};

    DynamixelState(const DynamixelState& state) = default;
    DynamixelState() = default;
  };

  explicit DynamixelServoNode(DynamixelState state);
  virtual ~DynamixelServoNode();

  void SetupServos();

  enum DirectionState {
    DirectionOptimal,
    DirectionCounterClockwise,
    DirectionClockwise
  };

private:
  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;

  DynamixelState state;

  rclcpp::Service<GetCurrent>::SharedPtr get_current_server_;
  rclcpp::Subscription<SetCurrentLimit>::SharedPtr set_current_limit_subscriber_; 
};

#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDR_PRESENT_CURRENT 126
#define ADDR_CURRENT_LIMIT 102
#define ADDR_VELOCITY_I_GAIN 76
#define ADDR_VELOCITY_P_GAIN 78
#define ADDR_POSITION_D_GAIN 80
#define ADDR_POSITION_I_GAIN 82
#define ADDR_POSITION_P_GAIN 84

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;

void DynamixelServoNode::SetupServos()
{
  for (auto& [id, servo] : state.servos)
  {
    // Set position p gain
    dxl_comm_result = packetHandler->write4ByteTxRx(
      portHandler,
      id,
      ADDR_POSITION_P_GAIN,
      servo.positionPGain,
      &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write position P gain: %d", id);
    }

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to write position P gain for ID %d: %s",
        id,
        packetHandler->getTxRxResult(dxl_comm_result)
      );
      return;
    }

    // Set position i gain
    dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler,
      id,
      ADDR_POSITION_I_GAIN,
      servo.positionIGain,
      &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to Set position I gain: %d", id);
    }

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to write position I gain for ID %d: %s",
        id,
        packetHandler->getTxRxResult(dxl_comm_result)
      );
      return;
    }

    // Set position d gain
    dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler,
      id,
      ADDR_POSITION_D_GAIN,
      servo.positionDGain,
      &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to Set position D gain: %d", id);
    }

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to write position D gain for ID %d: %s",
        id,
        packetHandler->getTxRxResult(dxl_comm_result)
      );
      return;
    }

    // Set velocity p gain
    dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler,
      id,
      ADDR_VELOCITY_P_GAIN,
      servo.velocityPGain,
      &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to Set velocity P gain: %d", id);
    }

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to write velocity P gain for ID %d: %s",
        id,
        packetHandler->getTxRxResult(dxl_comm_result)
      );
      return;
    }

    // Set velocity i gain
    dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler,
      id,
      ADDR_VELOCITY_I_GAIN,
      servo.velocityIGain,
      &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to Set velocity I gain: %d", id);
    }

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to write velocity I gain for ID %d: %s",
        id,
        packetHandler->getTxRxResult(dxl_comm_result)
      );
      return;
    }

    // Set current limit
    dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler,
      id,
      ADDR_CURRENT_LIMIT,
      servo.currentLimit,
      &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write current limit: %d", id);
    }

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to write current limit for ID %d: %s",
        id,
        packetHandler->getTxRxResult(dxl_comm_result)
      );
      return;
    }
  }
}

DynamixelServoNode::DynamixelServoNode(DynamixelState state) : 
  Node("read_write_node"),
  state(state)
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
      static_cast<dynamixel_id_t>(msg->id),
      ADDR_PRESENT_POSITION,
      reinterpret_cast<uint32_t *>(&present_position),
      &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read present position for ID: %d", msg->id);
      return;
    }

    if (dxl_comm_result != COMM_SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to read present position for ID %d: %s",
        msg->id,
        packetHandler->getTxRxResult(dxl_comm_result)
      );
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
      int present_position = 0;
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler,
        (uint8_t) request->id,
        ADDR_PRESENT_POSITION,
        reinterpret_cast<uint32_t *>(&present_position),
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to read present position for ID %d: %s",
          request->id,
          packetHandler->getTxRxResult(dxl_comm_result)
        );
        return;
      }

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Position: %d]",
        request->id,
        present_position
      );

      response->position = present_position;
    };

  get_position_server_ = create_service<GetPosition>("get_position", get_present_position);

  auto get_present_current =
    [this](
      const std::shared_ptr<GetCurrent::Request> request,
      std::shared_ptr<GetCurrent::Response> response) -> void
    {
      uint16_t raw_current = 0;
      dxl_error = 0;

      // Present Current is 2 bytes, signed, in mA units
      dxl_comm_result = packetHandler->read2ByteTxRx(
        portHandler,
        static_cast<uint8_t>(request->id),
        ADDR_PRESENT_CURRENT,
        &raw_current,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to read present current for ID %d: %s",
          request->id,
          packetHandler->getTxRxResult(dxl_comm_result)
        );
        response->current = 0;
        return;
      }

      if (dxl_error != 0) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Present current read error for ID %d: %s",
          request->id,
          packetHandler->getRxPacketError(dxl_error)
        );
        response->current = 0;
        return;
      }

      // Cast raw 16-bit to signed mA value
      int16_t signed_current = static_cast<int16_t>(raw_current);
      response->current = signed_current;

      RCLCPP_INFO(
        this->get_logger(),
        "Get [ID: %d] [Present Current: %d mA]",
        request->id,
        static_cast<int>(signed_current)
      );
    };

  get_current_server_ = create_service<GetCurrent>("get_current", get_present_current);

    // NEW: set_current_limit subscriber
  set_current_limit_subscriber_ =
    this->create_subscription<SetCurrentLimit>(
      "set_current_limit",
      QOS_RKL10V,
      [this](const SetCurrentLimit::SharedPtr msg) -> void
      {
        uint8_t dxl_error_local = 0;

        // Clamp requested limit into valid range [0, 1750] mA (XL330 spec)
        int32_t requested = msg->current_limit;
        if (requested < 0) {
          requested = 0;
        }
        if (requested > 1750) {
          requested = 1750;
        }
        uint16_t limit_mA = static_cast<uint16_t>(requested);

        // 1) Disable torque (EEPROM write requires torque off)
        dxl_comm_result = packetHandler->write1ByteTxRx(
          portHandler,
          static_cast<uint8_t>(msg->id),
          ADDR_TORQUE_ENABLE,
          0,
          &dxl_error_local
        );

        if (dxl_comm_result != COMM_SUCCESS || dxl_error_local != 0) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Failed to disable torque before setting current limit (ID %d): %s",
            msg->id,
            packetHandler->getTxRxResult(dxl_comm_result)
          );
          return;
        }

        // 2) Write CURRENT_LIMIT (2 bytes, mA)
        dxl_error_local = 0;
        dxl_comm_result = packetHandler->write2ByteTxRx(
          portHandler,
          static_cast<uint8_t>(msg->id),
          ADDR_CURRENT_LIMIT,
          limit_mA,
          &dxl_error_local
        );

        if (dxl_comm_result != COMM_SUCCESS) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Failed to set current limit for ID %d: %s",
            msg->id,
            packetHandler->getTxRxResult(dxl_comm_result)
          );
        } else if (dxl_error_local != 0) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Current limit write error for ID %d: %s",
            msg->id,
            packetHandler->getRxPacketError(dxl_error_local)
          );
        } else {
          RCLCPP_INFO(
            this->get_logger(),
            "Set [ID: %d] [Current Limit: %d mA]",
            msg->id,
            static_cast<int>(limit_mA)
          );
        }

        // 3) Re-enable torque
        dxl_error_local = 0;
        dxl_comm_result = packetHandler->write1ByteTxRx(
          portHandler,
          static_cast<uint8_t>(msg->id),
          ADDR_TORQUE_ENABLE,
          1,
          &dxl_error_local
        );

        if (dxl_comm_result != COMM_SUCCESS || dxl_error_local != 0) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Warning: failed to re-enable torque after setting current limit (ID %d)",
            msg->id
          );
        }
      }
    );


  SetupServos();
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
  DynamixelServoNode::DynamixelState state = {};

  DynamixelServoNode::DynamixelServo servo1 = {};
  DynamixelServoNode::DynamixelServo servo2 = {};

  state.servos[3] = servo1;
  state.servos[4] = servo2;
  
  portHandler = dynamixel::PortHandler::getPortHandler(state.deviceName.c_str());
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
  dxl_comm_result = portHandler->setBaudRate(state.baudRate);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("Dynamixel_Servo_Node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("Dynamixel_Servo_Node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<DynamixelServoNode>(state);
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
