#include <iostream>
#include <fcntl.h>      // File control definitions
#include <unistd.h>     // UNIX standard function definitions
#include <termios.h>    // POSIX terminal control definitions
#include <cstring>      // For memset
#include <sstream>      // For stringstream
#include <sys/select.h>
#include <cstdint>      // Include this header for uint8_t

#define HEADER_BYTE 0xA5
#define MSG_SERVO_SET_POSITION 0x00  // Message ID for ServoSetPosition

void sendServoSetPositionMessage(int serial_port, uint8_t id, float radians, bool is_counterclockwise);

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
    uint8_t id;
    float deg;
    bool is_counterclockwise;

    while (true) {
        std::cout << "Enter ID: ";
        std::cin >> id;  // Read the ID
        std::cout << "Enter position (in degrees): ";
        std::cin >> deg; // Read the position (in degrees)
        std::cout << "Enter direction (0 for CW, 1 for CCW): ";
        std::cin >> is_counterclockwise; // Read the direction

        // Convert degrees to radians
        float radians = (deg * 3.14159265f) / 180.0f;

        // Send the ServoSetPosition message to the Arduino
        sendServoSetPositionMessage(serial_port, id, radians, is_counterclockwise);

        // Optionally, add a small delay to avoid overloading the Arduino
        usleep(500000);  // Sleep for 0.5 seconds
    }

    // Close the serial port (this will never be reached as the loop is infinite)
    close(serial_port);
    return 0;
}

void sendServoSetPositionMessage(int serial_port, uint8_t id, float radians, bool is_counterclockwise) {
    // Message size: 1 byte for HEADER + 1 byte for message_id + 1 byte for ID and flags + sizeof(float) for radians
    std::cout << "Serial Port: " << serial_port << '\t' << "ID: " << id << '\t' 
              << "Radians: " << radians << '\t' << "Direction: " << is_counterclockwise << '\n';

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
}