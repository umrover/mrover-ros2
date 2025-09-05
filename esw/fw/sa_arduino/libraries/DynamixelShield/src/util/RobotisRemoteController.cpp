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

#include "RobotisRemoteController.h"

#ifdef SoftwareSerial_h
RobotisRemoteController::RobotisRemoteController(uint8_t rx_pin, uint8_t tx_pin)
  : p_hw_port_(nullptr), stream_port_(nullptr), enable_hw_port_(false)
{
  p_sw_port_ = new SoftwareSerial(rx_pin, tx_pin);
  stream_port_ = (Stream*) p_sw_port_;

  memset(&rc100_rx_, 0, sizeof(rc100_rx_));
}
#else
// #pragma message("\r\nWarning : You CAN NOT use SoftwareSerial version of the RobotisRemoteController function(only HardwareSerial version enabled), because this board DOES NOT SUPPORT SoftwareSerial.h")
// #pragma message("\r\nWarning : Please use RobotisRemoteController(HardwareSerial&) constructor only. (eg. RobotisRemoteController rc(Serial1);")
#endif

RobotisRemoteController::RobotisRemoteController(HardwareSerial& port)
  : p_hw_port_(&port), stream_port_(nullptr), enable_hw_port_(true)
{
#ifdef SoftwareSerial_h
  p_sw_port_ = nullptr;
#endif
  stream_port_ = (Stream*) p_hw_port_;
  memset(&rc100_rx_, 0, sizeof(rc100_rx_));
  begin();
}

RobotisRemoteController::~RobotisRemoteController()
{
}

void RobotisRemoteController::begin(uint32_t baudrate)
{
  rc100_rx_.state = 0;
  rc100_rx_.index = 0;
  rc100_rx_.received = false;
  rc100_rx_.released_event = false;

  if(enable_hw_port_){
    p_hw_port_ != nullptr ? p_hw_port_->begin(baudrate) : (void)(p_hw_port_);
  }else{
#ifdef SoftwareSerial_h
    p_sw_port_ != nullptr ? p_sw_port_->begin(baudrate) : (void)(p_sw_port_);
#endif  
  }
}

bool RobotisRemoteController::availableData(void)
{
  bool ret = false;

  if (stream_port_ == nullptr)
    return false;

  while(stream_port_->available() > 0)
  {
    ret = rc100Update(stream_port_->read());
  }

  return ret;
}

uint16_t RobotisRemoteController::readData(void)
{
  return rc100_rx_.data;
}

bool RobotisRemoteController::availableEvent(void)
{
  rc100_rx_.released_event = false;

  if (stream_port_ == nullptr)
    return false;

  while(stream_port_->available() > 0)
  {
    rc100Update(stream_port_->read());
  }
  
  return rc100_rx_.released_event;
}

uint16_t RobotisRemoteController::readEvent(void)
{
  return rc100_rx_.event_msg;
}

void RobotisRemoteController::flushRx(void)
{
  if (stream_port_ == nullptr)
    return;

  while(stream_port_->available())
  {
    stream_port_->read();
  }  
}

int RobotisRemoteController::available()
{
  if (stream_port_ == nullptr)
    return 0;

  return stream_port_->available();
}

int RobotisRemoteController::peek()
{
  if (stream_port_ == nullptr)
    return -1;

  return stream_port_->peek();
}

void RobotisRemoteController::flush()
{
  if (stream_port_ == nullptr)
    return;

  stream_port_->flush();
}

int RobotisRemoteController::read()
{
  if (stream_port_ == nullptr)
    return -1;

  return stream_port_->read();
}

size_t RobotisRemoteController::write(uint8_t data)
{
  if (stream_port_ == nullptr)
    return 0;

  return stream_port_->write(data);
}




bool RobotisRemoteController::rc100Update(uint8_t data)
{
  bool ret = false;
  static uint8_t save_data;
  static uint8_t inv_data;
  static uint32_t pre_time;

  inv_data = ~data;

  if (millis()-pre_time > 100)
  {
    rc100_rx_.state = 0;
  }

  switch(rc100_rx_.state)
  {
    case 0:
      if (data == 0xFF)
      {
        pre_time = millis();
        rc100_rx_.state = 1;
      }
      break;

    case 1:
      if (data == 0x55)
      {
        rc100_rx_.state    = 2;
        rc100_rx_.received = false;
        rc100_rx_.data     = 0;
      }
      else
      {
        rc100_rx_.state = 0;
      }
      break;

    case 2:
      rc100_rx_.data  = data;
      save_data      = data;
      rc100_rx_.state = 3;
      break;

    case 3:
      if (save_data == inv_data)
      {
        rc100_rx_.state = 4;
      }
      else
      {
        rc100_rx_.state = 0;
      }
      break;

    case 4:
      rc100_rx_.data |= data<<8;
      save_data      = data;
      rc100_rx_.state = 5;
      break;

    case 5:
      if (save_data == inv_data)
      {
        if (rc100_rx_.data > 0)
        {
          rc100_rx_.event_msg = rc100_rx_.data;
        }
        else
        {
          rc100_rx_.released_event = true;
        }
        rc100_rx_.received = true;
        ret = true;
      }
      rc100_rx_.state = 0;
      break;

    default:
      rc100_rx_.state = 0;
      break;
  }

  return ret;
}
