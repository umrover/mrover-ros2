#include <DynamixelShield.h>
#include <DFRobot_SHT20.h>

#include "temp_sensor.h"

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;
DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);
TempSensor temp_sensor;

// int raw_position_value = 0;
int raw_position_value = dxl.getPresentPosition(DXL_ID);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup(){
  //Serial.begin(9600);
  sht20.initSHT20();
  delay(100);
  temp_sensor.setup();


  // DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(DXL_ID);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, 4); // Operating Mode 4 is Extended Position Control Mode
  dxl.torqueOn(DXL_ID);
}

void loop(){
  float temp = sht20.readTemperature();

  float thermistorValue = temp_sensor.getTemperature(); 
  int rawVal = temp_sensor.getRawData();

  float humidity = sht20.readHumidity() / 100.0;

  delay(1000);

  // Spins servo 45 degrees
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in DEGREE value
  //dxl.setGoalPosition(DXL_ID, angle, UNIT_DEGREE);
  // Print present position in raw value
  dxl.setGoalPosition(DXL_ID, raw_position_value);
  delay(1000);
  // Print present position in raw and degree value
  DEBUG_SERIAL.print("Present Position(raw) : ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID));
  DEBUG_SERIAL.print("Present Position(degree) : ");
  DEBUG_SERIAL.println(dxl.getPresentPosition(DXL_ID, UNIT_DEGREE));
  delay(1000);
  
  raw_position_value += 511;
}

