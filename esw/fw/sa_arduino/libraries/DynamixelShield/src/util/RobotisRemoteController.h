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

#ifndef RC100_H_
#define RC100_H_

#include <Arduino.h>

#ifdef ARDUINO_ARCH_AVR
#include <SoftwareSerial.h>
#endif

////////// define RC-100 button key value ////////////////
#define RC100_BTN_U		(1)
#define RC100_BTN_D		(2)
#define RC100_BTN_L		(4)
#define RC100_BTN_R		(8)
#define RC100_BTN_1		(16)
#define RC100_BTN_2		(32)
#define RC100_BTN_3		(64)
#define RC100_BTN_4		(128)
#define RC100_BTN_5		(256)
#define RC100_BTN_6		(512)

#define PACKET_LENGTH 		6

class RobotisRemoteController : public Stream{
  public:
#ifdef SoftwareSerial_h  
    RobotisRemoteController(uint8_t rx_pin=7, uint8_t tx_pin=8);
#endif    
    RobotisRemoteController(HardwareSerial& port);
    virtual ~RobotisRemoteController();

    /**
     * @brief Initialization function to start communication with RC.
     *      Or change baudrate of baud serial port.
     * @code
     * RobotisRemoteController rc;
     * rc.begin();
     * rc.begin(115200);
     * @endcode
     * @param baud The port speed you want on the board (the speed to communicate with RC) (default : 57600)
     */    
    void begin(uint32_t baudrate = 57600);

    /**
     * @brief Check whether there is an RC packet normally received.
     * @code
     * RobotisRemoteController rc;
     * rc.begin();
     * if(rc.availableData()){
     *   Serial.println(rc.readData());
     * }
     * @endcode
     * @return True(1) if it exists, false(0) otherwise.
     */    
    bool availableData(void);

    /**
     * @brief Read data of RC packet.
     * @code
     * RobotisRemoteController rc;
     * rc.begin();
     * if(rc.availableData()){
     *   Serial.println(rc.readData());
     * }
     * @endcode
     * @return data of RC packet.
     */    
    uint16_t readData(void);

    /**
     * @brief Check whether there is an RC packet normally received.
     *       However, this only works after receiving the release event packet (data: 0).
     *       That is, it checks whether there is a packet when the button is released and then pressed again on the RC100 controller.
     * @code
     * RobotisRemoteController rc;
     * rc.begin();
     * if(rc.availableEvent()){
     *   Serial.println(rc.readEvent());
     * }
     * @endcode
     * @return True(1) if it exists, false(0) otherwise.
     */   
    bool availableEvent(void);

    /**
     * @brief Read data of RC packet. (Use with availableEvent() function)
     * @code
     * RobotisRemoteController rc;
     * rc.begin();
     * if(rc.availableEvent()){
     *   Serial.println(rc.readEvent());
     * }
     * @endcode
     * @return data of RC packet.
     */        
    uint16_t readEvent(void);

    /**
     * @brief Clear Serial RX buffer.
     * @code
     * RobotisRemoteController rc;
     * rc.begin();
     * rc.flushRx();
     * @endcode
     */
    void flushRx(void);

    // Stream
    virtual int available() override;
    virtual int read() override;
    virtual int peek() override;
    
    // Print
    virtual void flush() override;
    virtual size_t write(uint8_t) override;
    using Print::write; // pull in write(str) and write(buf, size) from Print

   
  private:
    typedef struct
    {
      uint8_t  state;
      uint8_t  index;
      bool     received;
      bool     released_event;
      uint16_t data;
      uint16_t event_msg;
    } rc100_t;

    rc100_t rc100_rx_;

#ifdef SoftwareSerial_h
    SoftwareSerial *p_sw_port_;
#endif
    HardwareSerial *p_hw_port_;
    Stream *stream_port_;

    bool enable_hw_port_;

    bool rc100Update(uint8_t data);
    bool rc100Receive(unsigned char *pPacket, int numPacket);
};

#endif /* RC100_H_ */
