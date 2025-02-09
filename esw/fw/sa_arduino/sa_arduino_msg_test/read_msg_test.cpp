#include <iostream>
#include <fcntl.h>      // File control definitions
#include <unistd.h>     // UNIX standard function definitions
#include <termios.h>    // POSIX terminal control definitions
#include <cstring>      // For memset
#include <sstream>      // For stringstream
#include <sys/select.h>
#include <cstdint>      // Include this header for uint8_t
#include <cmath>

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


void readServoSetPositionMessage(int serial_port);

int main() {
    const char *device = "/dev/ttyACM0"; // Replace with the appropriate device
    int baudrate = B115200;               // Match the Arduino's0 baud rate

    // Open the serial port
    int serial_port = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_port < 0) {
        std::cerr << "Error: Unable to open serial port " << device << std::endl;
        return 1;
    }

    // Configure the serial port
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(serial_port, &tty) != 0) {
        std::cerr << "Error: Unable to get terminal attributes" << std::endl;
        close(serial_port);
        return 1;
    }

    // Set baud rate
    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    // 8N1 mode: 8 data bits, no parity, 1 stop bit
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;             // No signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;             // No remapping, no delays
    tty.c_cc[VMIN] = 1;          // Read blocks until at least 1 byte is available
    tty.c_cc[VTIME] = 1;         // Timeout for read

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control
    tty.c_cflag |= (CLOCAL | CREAD);         // Enable receiver, local mode
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS); // No parity, 1 stop bit, no flow control

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cerr << "Error: Unable to set terminal attributes" << std::endl;
        close(serial_port);
        return 1;
    }

    // Continuously read numbers from user and send them to the Arduino

    while (true) {

        // Send the ServoSetPosition message to the Arduino
        readServoSetPositionMessage(serial_port);

        // Optionally, add a small delay to avoid overloading the Arduino
        usleep(1000000);  // Sleep for 0.2 seconds
    }

    // Close the serial port (this will never be reached as the loop is infinite)
    close(serial_port);
    return 0;
}

void readServoSetPositionMessage(int serial_port) {

    // Read from serial_port and print out the HEADER_BYTE, message_id, Servo ID, present position

    uint8_t read_buffer[2 + MSG_SERVO_POSITION_DATA_SIZE];
    ssize_t bytes_read = read(serial_port, &read_buffer, sizeof(read_buffer)); // temp/humidity double check the -1

    if ((read_buffer[0] == HEADER_BYTE) && (read_buffer[1] == MSG_SERVO_POSITION_DATA)) {
        std::cout << "read_buffer[0] (HEADER_BYTE): " << (int)read_buffer[0] 
              << "\tread_buffer[1] (message_id): " << (int)read_buffer[1] 
              << "\tread_buffer[2] (DXL_ID): " << (int)read_buffer[2] << '\n';

        // Pack radians as float (4 bytes)
        // memcpy(&read_buffer[3], &radians, sizeof(float));
        float radians;
        memcpy(&radians, &read_buffer[3], sizeof(float));
        float degrees = (radians * 180.0) / PI;

        // Debugging: Print the raw bytes for radians
        std::cout << "read_buffer[3] (Radian bytes): ";
        for (size_t i = 3; i < sizeof(read_buffer); ++i) {
            std::cout << (int)read_buffer[i] << " ";
        }
        std::cout << '\n';

        std::cout << "Absolute Position:\tRadians: " << radians << "\tDegrees: " << degrees << "\n";
        std::cout << "Relative Position:\tRadians: " << fmod(radians, 2.0*PI) << "\tDegrees: " << fmod(degrees, 360.0) << "\n\n";

    }

    

    /*
    // old write code vvvv
    uint8_t message[1 + 1 + 1 + sizeof(float)];

    // Set the HEADER_BYTE and message_id
    message[0] = HEADER_BYTE;
    message[1] = MSG_SERVO_SET_POSITION;  // Set message_id

    // Pack the ID and is_counterclockwise flag into the third byte
    uint8_t id_flags = (id & 0x0F) << 1 | (is_counterclockwise & 0x01);
    message[2] = id_flags;  // Set ID and direction (CW/CCW)

    // Debugging: Print out message bytes so we can see what is being sent
    std::cout << "Message[0] (HEADER_BYTE): " << (int)message[0] 
              << "\tMessage[1] (message_id): " << (int)message[1] 
              << "\tMessage[2] (ID + direction): " << (int)message[2] << '\n';

    // Pack radians as float (4 bytes)
    memcpy(&message[3], &radians, sizeof(float));

    // Debugging: Print the raw bytes for radians
    std::cout << "Message[3] (Radian bytes): ";
    for (size_t i = 3; i < sizeof(message); ++i) {
        std::cout << (int)message[i] << " ";
    }
    std::cout << '\n';

    // Send the message to the serial port
    ssize_t bytes_written = write(serial_port, message, sizeof(message));
    if (bytes_written < 0) {
        std::cerr << "Error: Unable to write to serial port" << std::endl;
    } else {
        std::cout << "Sent message to set position with ID: " << id << ", Radians: " << radians 
                  << ", Direction: " << (is_counterclockwise ? "CCW" : "CW") << "\n\n";
    }
    */
}