/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

DynamixelShield dxl;
DynamixelShield dxl_1;
DynamixelShield dxl_2;
DynamixelShield dxl_3;
DynamixelShield dxl_4;
DynamixelShield dxl_5;

const uint8_t DXL_ID_1 = 1;
const uint8_t DXL_ID_2 = 2;
const uint8_t DXL_ID_3 = 3;
const uint8_t DXL_ID_4 = 4;
const uint8_t DXL_ID_5 = 5;

const float DXL_PROTOCOL_VERSION = 2.0;

// int raw_position_value = 0;
int raw_position_value_1 = dxl.getPresentPosition(DXL_ID_1);
int raw_position_value_2 = dxl.getPresentPosition(DXL_ID_1);
int raw_position_value_3 = dxl.getPresentPosition(DXL_ID_1);
int raw_position_value_4 = dxl.getPresentPosition(DXL_ID_1);
int raw_position_value_5 = dxl.getPresentPosition(DXL_ID_1);



//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl_1.begin(57600);
  dxl_2.begin(57600);
  dxl_3.begin(57600);
  dxl_4.begin(57600);
  dxl_5.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl_1.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_2.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_3.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_4.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  dxl_5.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl_1.ping(DXL_ID_1);
  dxl_2.ping(DXL_ID_2);
  dxl_3.ping(DXL_ID_3);
  dxl_4.ping(DXL_ID_4);
  dxl_5.ping(DXL_ID_5);

  // Turn off torque when configuring items in EEPROM area
  dxl_1.torqueOff(DXL_ID_1);
  dxl_1.setOperatingMode(DXL_ID_1, 4); // Operating Mode 4 is Extended Position Control Mode
  dxl_1.torqueOn(DXL_ID_1);

  dxl_2.torqueOff(DXL_ID_2);
  dxl_2.setOperatingMode(DXL_ID_2, 4); // Operating Mode 4 is Extended Position Control Mode
  dxl_2.torqueOn(DXL_ID_2);

  dxl_3.torqueOff(DXL_ID_3);
  dxl_3.setOperatingMode(DXL_ID_3, 4); // Operating Mode 4 is Extended Position Control Mode
  dxl_3.torqueOn(DXL_ID_3);

  dxl_4.torqueOff(DXL_ID_4);
  dxl_4.setOperatingMode(DXL_ID_4, 4); // Operating Mode 4 is Extended Position Control Mode
  dxl_4.torqueOn(DXL_ID_4);

  dxl_5.torqueOff(DXL_ID_5);
  dxl_5.setOperatingMode(DXL_ID_5, 4); // Operating Mode 4 is Extended Position Control Mode
  dxl_5.torqueOn(DXL_ID_5);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in DEGREE value
  //dxl.setGoalPosition(DXL_ID, angle, UNIT_DEGREE);
  // Print present position in raw value
  dxl_1.setGoalPosition(DXL_ID_1, raw_position_value_1);
  dxl_2.setGoalPosition(DXL_ID_2, raw_position_value_2);
  dxl_3.setGoalPosition(DXL_ID_3, raw_position_value_3);
  dxl_4.setGoalPosition(DXL_ID_4, raw_position_value_4);
  dxl_5.setGoalPosition(DXL_ID_5, raw_position_value_5);
  delay(1000);
  
  // Print present position in raw and degree value
  DEBUG_SERIAL.print("Present Position(raw) : ");
  DEBUG_SERIAL.println(dxl_1.getPresentPosition(DXL_ID_1));
  DEBUG_SERIAL.print("Present Position(degree) : ");
  DEBUG_SERIAL.println(dxl_1.getPresentPosition(DXL_ID_1, UNIT_DEGREE));
  delay(1000);
  
  raw_position_value_1 += 511;
  raw_position_value_2 += 511;
  raw_position_value_3 += 511;
  raw_position_value_4 += 511;
  raw_position_value_5 += 511;
}
