#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_sdk/port_handler.h>
#include <dynamixel_sdk/packet_handler.h>

// Now you can use Dynamixel SDK classes
using namespace dynamixel;

// Example usage:
PortHandler *port_handler = PortHandler::getPortHandler("some device");