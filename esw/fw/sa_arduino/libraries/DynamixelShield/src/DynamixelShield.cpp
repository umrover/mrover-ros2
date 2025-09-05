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


#include "DynamixelShield.h"


DynamixelShield::DynamixelShield(HardwareSerial& port, int dir_pin)
  : Dynamixel2Arduino(port, dir_pin)
{}

DynamixelShield::~DynamixelShield()
{}


/* deprecated functions(Will be removed later) */
//recommended alternative function : bool setBaudrate(uint8_t id, uint32_t baudrate);
bool DynamixelShield::setBaud(uint8_t id, uint32_t new_baud)
{
  return setBaudrate(id, new_baud);
}

//recommended alternative function : bool setOperatingMode(uint8_t id, uint8_t mode);
bool DynamixelShield::setJointMode(uint8_t id)
{
  return setOperatingMode(id, OP_POSITION);
}

//recommended alternative function : bool setOperatingMode(uint8_t id, uint8_t mode);
bool DynamixelShield::setWheelMode(uint8_t id)
{
  return setOperatingMode(id, OP_VELOCITY);
}

//recommended alternative function : float getPresentPosition(uint8_t id, uint8_t unit = UNIT_RAW); 
int32_t DynamixelShield::getCurPosition(uint8_t id)
{
  return (int32_t)getPresentPosition(id);
}

//recommended alternative function : bool setGoalVelocity(uint8_t id, float value, uint8_t unit = UNIT_RAW);
bool DynamixelShield::setGoalSpeed(uint8_t id, int32_t speed)
{ 
  return setGoalVelocity(id, (float)speed);
}

//recommended alternative function : float getPresentVelocity(uint8_t id, uint8_t unit = UNIT_RAW);  
int32_t DynamixelShield::getCurSpeed(uint8_t id)
{
  return (int32_t)getPresentVelocity(id);
}

//recommended alternative function : bool setGoalPosition(uint8_t id, float value, UNIT_DEGREE);
bool DynamixelShield::setGoalAngle(uint8_t id, int32_t angle)
{
  return setGoalPosition(id, (float)angle, UNIT_DEGREE);
}

//recommended alternative function : float getPresentPosition(uint8_t id, UNIT_DEGREE);  
int32_t DynamixelShield::getCurAngle(uint8_t id)
{
  return (int32_t)getPresentPosition(id, UNIT_DEGREE);
}