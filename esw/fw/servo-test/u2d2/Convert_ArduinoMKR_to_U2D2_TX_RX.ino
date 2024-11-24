#include <DynamixelShield.h>

#define COMPUTER_USB Serial
#define DXL_PACKET_BUFFER_LENGTH  1024

uint8_t packet_buffer[DXL_PACKET_BUFFER_LENGTH];
const uint32_t GLOBAL_BAUDRATE = 57600;
unsigned int mbedTXdelayus = round(24000000 / GLOBAL_BAUDRATE);
unsigned long led_update_time = 0;

DynamixelShield dxl;

void setup() {
  // put your setup code here, to run once:
  // set USB connection for communication with Dynamixel Wizard 2.0
  COMPUTER_USB.begin(57600);

  // set DXL direction pin to baseline
  digitalWrite(DXL_DIR_PIN, LOW);
  while(digitalRead(DXL_DIR_PIN) != LOW);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
}

void loop() {
  // put your main code here, to run repeatedly:
  dataTransceiver();
}

void dataTransceiver()
{
  int length = 0;
  int i = 0;

  // USB -> DXL (set dxl direction pin high during TX to dxl)
  length = COMPUTER_USB.available();
  if( length > 0 )
  {
    digitalWrite(DXL_DIR_PIN, HIGH);
    while(digitalRead(DXL_DIR_PIN) != HIGH);
    for(i = 0; i < length; i++)
    {
      DXL_SERIAL.write(COMPUTER_USB.read());
    }
    DXL_SERIAL.flush();
    #if defined(ARDUINO_ARCH_MBED)
      delayMicroseconds(mbedTXdelayus);
    #endif
    digitalWrite(DXL_DIR_PIN, LOW);
    while(digitalRead(DXL_DIR_PIN) != LOW);
  }

  // DXL -> USB
  length = DXL_SERIAL.available();
  if( length > 0 )
  {
    if( length > DXL_PACKET_BUFFER_LENGTH )
    {
      length = DXL_PACKET_BUFFER_LENGTH;
    }
    for(i = 0; i < length; i++)
    {
      packet_buffer[i] = DXL_SERIAL.read();
    }
    COMPUTER_USB.write(packet_buffer, length);
  }
}
