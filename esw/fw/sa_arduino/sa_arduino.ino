#include <DynamixelShield.h>
//#include <DFRobot_SHT20.h>

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
void processServoSetPosition(uint8_t *buffer);
void processServoPositionData(uint8_t *buffer);
//void processTemperatureHumidityData();


// Sensor/Servo Constants
const uint8_t DXL_ID = 1;
const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;
//DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);
TempSensor temp_sensor;

//This namespace is required to use DYNAMIXEL Control table item names
using namespace ControlTableItem;

void setup(){
  //Serial.begin(9600);
  //sht20.initSHT20();
  delay(100);
  temp_sensor.setup();

  Serial.begin(115200); // Match the baud rate with the C++ program

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
}

void loop(){

  if (Serial.available() >= 2) {  // ensure we have at least header (1) & message_id (1)
    uint8_t header = Serial.read();  
    uint8_t message_id = Serial.read();  

    if (header == HEADER_BYTE) {

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
            /*  
            // NOTE: for some reason, this file won't compile with the sensor stuff
            // It claims to not find the library so we can fix this later when moving files probably
            case MSG_TEMPERATURE_HUMIDITY:
                processSensorData(buffer);
                break;
            */
        }
      }

    }

  }

}


// Process Different Message Types

void processServoSetPosition(uint8_t *buffer) {
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

    float diff = set_degrees - present_degrees;

    // TODO: this logic is close bur definitely not right
    // It will end in correct position but may go in wrong direction or wrap unnecessarily
    // EXAMPLE: start at 180, then go to 270 CW (it goes CCW instead)

    if (diff > 0) {
      if (is_counterclockwise) {
        dxl.setGoalPosition(id, set_degrees, UNIT_DEGREE);
      } else {
        dxl.setGoalPosition(id, set_degrees + 360, UNIT_DEGREE);
      }
    } else {
      if (is_counterclockwise) {
        dxl.setGoalPosition(id, set_degrees + 360, UNIT_DEGREE);
      } else {
        dxl.setGoalPosition(id, set_degrees, UNIT_DEGREE);
      }
    }
    
}

void processServoPositionData(uint8_t *buffer) {

    // NOTE: not 100% sure when we are supposed to send data
    // this is based off Cindy's work but idk where it should go fs

    uint8_t id = buffer[0] & 0x0F;  // [4b unused][4b id]

    dxl.ping(id);
    float presentDeg = dxl.getPresentPosition(id, UNIT_DEGREE);
    float presentRad = presentDeg * PI / 180.0;

    uint8_t radBytes[sizeof(float)];
    memcpy(radBytes, &presentRad, sizeof(float));

    Serial.write((byte)HEADER_BYTE);
    Serial.write((byte)MSG_SERVO_POSITION_DATA);
    Serial.write((byte)id);
    Serial.write(radBytes, sizeof(float));
}

/*
void processTemperatureHumidityData() {
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

*/






/*
  // old loop code for reference
  
  float temp = sht20.readTemperature();

  float thermistorValue = temp_sensor.getTemperature(); 
  int rawVal = temp_sensor.getRawData();

  float humidity = sht20.readHumidity() / 100.0;

  // START OUR CODE HERE!

  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read the incoming data until newline
    input.trim();  // Remove extra whitespace or newline characters

    // Check if the input is not empty
    if (input.length() > 0) {
      int spaceIndex = input.indexOf(' ');  // Find the space separating ID and Degrees
      if (spaceIndex != -1) {
        // Extract ID and Degrees from the input string
        int id = input.substring(0, spaceIndex).toInt();  // Get the ID (before the space)
        int pos_new = input.substring(spaceIndex + 1).toInt();  // Get Degrees (after the space)
        int direction = input.substring(spaceIndex + 2).toInt();



        // example numbers
        int id = 0;
        bool is_counterclockwise = 0; // True:CCW:Positive // False:CW:Negative
        float radians = 4.5;

        // Validate parsed values
        if (id >= 0 || input.substring(spaceIndex + 1) == "0") {

          // Get DYNAMIXEL information
          dxl.ping(id);
          
          // Turn off torque when configuring items in EEPROM
          // Operating Mode 4 is Extended Position Control Mode
          dxl.torqueOff(id);
          dxl.setOperatingMode(id, 4);
          dxl.torqueOn(id);

          // TODO: use isCCW and radians to set position
          float degrees = (radians * 180.0)/PI;
          if (!is_counterclockwise)
          {
            degrees*=-1;
          }
          dxl.setGoalAngle(id, degrees);

          // TODO: convert current position to radians and send out

          presentDeg = dxl.getCurAngle(id);
          uint32_t presentRad = presentDeg*PI/180.0;
          Serial.write((byte)HEADER_BYTE);
          Serial.write((byte)0x01); // message header
          Serial.write((byte)id);
          Serial.write(presentRad);

        }
      }
    } 
  } 
  */
