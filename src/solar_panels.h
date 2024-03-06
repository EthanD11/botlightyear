#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#define DXL_ID 6 // ID of the Dynamixel servo you want to control
#define BAUDRATE 57600 // Baudrate for serial communication
#define DEVICENAME "/dev/ttyAMA0" // Serial device name

// Dynamixel protocol definitions
#define INST_WRITE_DATA 0x03
#define ADDR_GOAL_POSITION 0x1E

void setBaudRate(int serial_fd, int baudrate);

void setGoalPosition(int serial_fd, int goal_position);