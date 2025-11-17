#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_sdk/port_handler.h>
#include <dynamixel_sdk/packet_handler.h>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/cmdline_parser.h>
#include <string>

#include <mrover/msg/set_position.hpp>
#include <mrover/srv/get_position.hpp>


// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"


class DynamixelServoNode : public rclcpp::Node
{
public:
  using SetPosition = mrover::msg::SetPosition;
  using GetPosition = mrover::srv::GetPosition;

  uint8_t dxl_error = 0;
  uint32_t goal_position = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  DynamixelServoNode();

private:
  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;

  int present_position;
};