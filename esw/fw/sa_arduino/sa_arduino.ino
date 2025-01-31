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


  Serial.begin(115200); // Match the baud rate with the C++ program

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
}

void loop(){
  float temp = sht20.readTemperature();

  float thermistorValue = temp_sensor.getTemperature(); 
  int rawVal = temp_sensor.getRawData();

  float humidity = sht20.readHumidity() / 100.0;

  // delay(1000);


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
        int pos_new = input.substring(spaceIndex + 1).toInt() % 360;  // Get Degrees (after the space)

        // Validate parsed values
        if (id >= 0 || input.substring(spaceIndex + 1) == "0") {

          // Get DYNAMIXEL information
          dxl.ping(id);
          
          // Turn off torque when configuring items in EEPROM
          // Operating Mode 4 is Extended Position Control Mode
          dxl.torqueOff(id);
          dxl.setOperatingMode(id, 4);
          dxl.torqueOn(id);


          // Adjust the motor position for the specified ID
          int pos_prev = dxl.getPresentPosition(id, UNIT_DEGREE) % 360;

          // now pos_new and pos_prev are DEG in range [0,359]
  
          // TODO: write logic to make sure servo moves in shortest path.
          



          int new_position = prev_position + (deg_in * (511 / 45));
          dxl.setGoalPosition(id, new_position);

          int delta = (new_position - prev_position);
          int temp = 0;

          while (abs(dxl.getPresentPosition(id)-prev_position) < abs(delta)) {
            // Serial.print("Present Position: ");
            // Serial.print(dxl.getPresentPosition(id));
            // Serial.print("\t\t");
            temp = dxl.getPresentPosition(id);
            delay(100);
          }

          // delay(1000);

          // Send back the ID and new position
          Serial.print("ID: ");
          Serial.print(id);
          Serial.print(", Current Position: ");
          Serial.print(temp);
          Serial.print(" Done");

        }
      }
    } 
  }

  // END OUR CODE HERE!




  /*

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

  */
}



/*

#include <DynamixelShield.h>

DynamixelShield dxl;
//const uint8_t DXL_ID = 0; // check label on Dynamixel

const float DXL_PROTOCOL_VERSION = 2.0;

using namespace ControlTableItem;


void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read the incoming data until newline
    input.trim();  // Remove extra whitespace or newline characters

    // Check if the input is not empty
    if (input.length() > 0) {
      int spaceIndex = input.indexOf(' ');  // Find the space separating ID and Degrees
      if (spaceIndex != -1) {
        // Extract ID and Degrees from the input string
        int id = input.substring(0, spaceIndex).toInt();  // Get the ID (before the space)
        int deg_in = input.substring(spaceIndex + 1).toInt();  // Get Degrees (after the space)

        // Validate parsed values
        if (id >= 0 && deg_in != 0 || input.substring(spaceIndex + 1) == "0") {

          // Get DYNAMIXEL information
          dxl.ping(id);
          
          // Turn off torque when configuring items in EEPROM
          // Operating Mode 4 is Extended Position Control Mode
          dxl.torqueOff(id);
          dxl.setOperatingMode(id, 4);
          dxl.torqueOn(id);


          // Adjust the motor position for the specified ID
          int prev_position = dxl.getPresentPosition(id);
          int new_position = prev_position + (deg_in * (511 / 45));
          dxl.setGoalPosition(id, new_position);

          int delta = (new_position - prev_position);
          int temp = 0;

          while (abs(dxl.getPresentPosition(id)-prev_position) < abs(delta)) {
            // Serial.print("Present Position: ");
            // Serial.print(dxl.getPresentPosition(id));
            // Serial.print("\t\t");
            temp = dxl.getPresentPosition(id);
            delay(100);
          }

          // delay(1000);

          // Send back the ID and new position
          Serial.print("ID: ");
          Serial.print(id);
          Serial.print(", Current Position: ");
          Serial.print(temp);
          Serial.print(" Done");

        }
      }
    } 
  }
}
*/