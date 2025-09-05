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

#ifndef DYNAMIXEL_SHIELD_H_
#define DYNAMIXEL_SHIELD_H_

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include "util/RobotisRemoteController.h"


#ifndef DYNAMIXEL_2_ARDUINO_H_
#error "\r\nWarning : To use DynamixelShield, you must install Dynamixel2Arduino library."
#error "\r\nWarning : Please search and install Dynamixel2Arduino in Arduino Library Manager. (For version dependencies, see http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/)"
#endif


// https://www.arduino.cc/reference/en/language/functions/communication/serial/
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_SAM_DUE)
  #define DXL_SERIAL   Serial
#else
  #define DXL_SERIAL   Serial1
#endif

#if defined(USE_ARDUINO_MKR_PIN_LAYOUT) || defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_PORTENTA_H7_M4)
  #define DXL_DIR_PIN		A6
#else
  #define DXL_DIR_PIN		2
#endif

class DynamixelShield : public Dynamixel2Arduino
{
//Most of the public functions of this class inherit the API of Dynamixel2Arduino.
//So, if you want to modify or view the code, please refer to the code in the Dynamixel2Arduino library. 
public:
  DynamixelShield(HardwareSerial& port = DXL_SERIAL, int dir_pin = DXL_DIR_PIN);
  ~DynamixelShield();


  /* Below are deprecated functions(Will be removed later) */

  //recommended alternative function : bool setBaudrate(uint8_t id, uint32_t baudrate);
  bool setBaud(uint8_t id, uint32_t new_baud); 

  //recommended alternative function : bool setOperatingMode(uint8_t id, uint8_t mode);
  bool setJointMode(uint8_t id);

  //recommended alternative function : bool setOperatingMode(uint8_t id, uint8_t mode);
  bool setWheelMode(uint8_t id);

  //recommended alternative function : float getPresentPosition(uint8_t id, uint8_t unit = UNIT_RAW); 
  int32_t getCurPosition(uint8_t id); 

  //recommended alternative function : bool setGoalVelocity(uint8_t id, float value, uint8_t unit = UNIT_RAW);
  bool setGoalSpeed(uint8_t id, int32_t speed); 

  //recommended alternative function : float getPresentVelocity(uint8_t id, uint8_t unit = UNIT_RAW);  
  int32_t getCurSpeed(uint8_t id); 

  //bool setGoalPosition(uint8_t id, float value, UNIT_DEGREE);
  bool setGoalAngle(uint8_t id, int32_t angle); 

  //float getPresentPosition(uint8_t id, UNIT_DEGREE);  
  int32_t getCurAngle(uint8_t id); 

private:

};

#endif 
