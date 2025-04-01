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


#define PI 3.1415926535897932384626433832795
#define HEADER_BYTE 0xA5

// Define message IDs
#define MSG_SERVO_SET_POSITION     0x00
#define MSG_SERVO_POSITION_DATA    0x01
#define MSG_TEMPERATURE_HUMIDITY   0x02

// Define message sizes (not including HEADER_BYTE or message_id)
#define MSG_SERVO_SET_POSITION_SIZE 5  // ID & is_counterclockwise (1) + Radians (4)
#define MSG_SERVO_POSITION_DATA_SIZE 5 // ID (1) + Radians (4)
#define MSG_TEMPERATURE_HUMIDITY_SIZE 8 // Temp (4) + Humidity (4)

// Maximum possible message size (largest struct)
#define MAX_MESSAGE_SIZE 8  

// Functions to handle messages
void readServoSetPosition(uint8_t *buffer);
void writeServoPositionData();
void writeTemperatureHumidityData();

/*
SHT3X Humidity and Temperature Sensor PinOut
      Black     GND
      Yellow    SCL (12)
      Green     SDA (11)
      Red       VCC
*/


// Sensor/Servo Constants
const uint8_t DXL_ID = 0;
const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;
DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);
TempSensor temp_sensor;

//This namespace is required to use DYNAMIXEL Control table item names
using namespace ControlTableItem;

void setup(){
  Serial.begin(115200);
  sht20.initSHT20();
  delay(100);
  temp_sensor.setup();


  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);

  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
}

void loop(){

  // send data here
  writeServoPositionData();
  writeTemperatureHumidityData();

  // check for set position request
  if (Serial.available() >= 2) {  // ensure we have at least header (1) & message_id (1)
    uint8_t header = Serial.read();  
    uint8_t message_id = Serial.read();  

    if ((header == HEADER_BYTE) && (message_id == MSG_SERVO_SET_POSITION)) {
      uint8_t buffer[MSG_SERVO_SET_POSITION_SIZE] = {0};
      Serial.readBytes(buffer, MSG_SERVO_SET_POSITION_SIZE);
      readServoSetPosition(buffer);
    } 

  }

  delay(100); // 100ms 

}


// Process Different Message Types

void readServoSetPosition(uint8_t *buffer) {
    uint8_t id_flags = buffer[0]; // [3b unused][4b id][1b is_counterclockwise]
    uint8_t id = (id_flags >> 1) & 0x0F;
    bool is_counterclockwise = id_flags & 0x01;

    float radians;
    memcpy(&radians, &buffer[1], sizeof(float));

    float set_degrees = (radians * 180.0) / PI;

    dxl.ping(id);
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, 4);
    dxl.torqueOn(id);

    float present_degrees = dxl.getPresentPosition(id, UNIT_DEGREE);

    // find difference in range [0,360]
    float diff = fmod(abs(set_degrees - present_degrees), 360);

    if (set_degrees > present_degrees) {
      if (is_counterclockwise) {
        dxl.setGoalPosition(id, present_degrees + diff, UNIT_DEGREE);
      } else {
        dxl.setGoalPosition(id, present_degrees - (360 - diff), UNIT_DEGREE); // (360+180) - (360-90) = (360+180) - 270 = -90 == 270 yay
      }
    } else {
      if (is_counterclockwise) {
        dxl.setGoalPosition(id, present_degrees + (360 - diff), UNIT_DEGREE);
      } else { 
        dxl.setGoalPosition(id, present_degrees - diff, UNIT_DEGREE);
      }
    }
    
}

void writeServoPositionData() {

    // 10Hz send position and temp/humidity data, this should be independant of the set position
    
    // TX and RX are seperate lines, do not interfere with each other

    dxl.ping(DXL_ID);
    float presentDeg = dxl.getPresentPosition(DXL_ID, UNIT_DEGREE);
    float presentRad = presentDeg * PI / 180.0;

    uint8_t radBytes[sizeof(float)];
    memcpy(radBytes, &presentRad, sizeof(float));

    Serial.write((byte)HEADER_BYTE);
    Serial.write((byte)MSG_SERVO_POSITION_DATA);
    Serial.write((byte)DXL_ID);
    Serial.write(radBytes, sizeof(float));
}


void writeTemperatureHumidityData() {
    float temp = sht20.readTemperature();
    float humidity = sht20.readHumidity() / 100.0;

    uint8_t tempBytes[sizeof(float)];
    uint8_t humidityBytes[sizeof(float)];
    memcpy(tempBytes, &temp, sizeof(float));
    memcpy(humidityBytes, &humidity, sizeof(float));

    Serial.write((byte)HEADER_BYTE);
    Serial.write((byte)MSG_TEMPERATURE_HUMIDITY);
    Serial.write(tempBytes, sizeof(float));
    Serial.write(humidityBytes, sizeof(float));
}



// %%%%%%%%%%%%%%%%%% GRAVEYARD %%%%%%%%%%%%%%%%%%%%%%%%

/*


    // old vvvv
      // determine message size to read in buffer
      uint8_t message_size = 0;
      switch (message_id) {
          case MSG_SERVO_SET_POSITION:
              message_size = MSG_SERVO_SET_POSITION_SIZE;
              break;
          case MSG_SERVO_POSITION_DATA:
              message_size = MSG_SERVO_POSITION_DATA_SIZE;
              break;
          case MSG_TEMPERATURE_HUMIDITY:
              message_size = MSG_TEMPERATURE_HUMIDITY_SIZE;
              break;
          default:
              return;
      }

      if (Serial.available() >= message_size) {  // ensure we have full message
        uint8_t buffer[MAX_MESSAGE_SIZE] = {0};
        Serial.readBytes(buffer, message_size);

        switch (message_id) {
            case MSG_SERVO_SET_POSITION:
                processServoSetPosition(buffer);
                break;
            case MSG_SERVO_POSITION_DATA:
                processServoPositionData(buffer);
                break;
            // NOTE: for some reason, this file won't compile with the sensor stuff
            // It claims to not find the library so we can fix this later when moving files probably
            case MSG_TEMPERATURE_HUMIDITY:
                processSensorData(buffer);
                break;
            
        }
      }

*/
