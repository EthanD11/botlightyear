#include "solar_panels.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <dynamixel_sdk.h>

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_MOVING_SPEED            32
#define ADDR_CW_ANGLE_LIMIT             6                  // JOINT MODE: neither are 0
#define ADDR_CCW_ANGLE_LIMIT            8                  // WHEEL MODE: both are 0

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting

#define DXL_ID                          6                  // Dynamixel ID
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyAMA0"      

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold


#define ESC_ASCII_VALUE                 0x1b


void deploy() {
  int port_num = portHandler(DEVICENAME);
  packetHandler();

  int index = 0;
  int dxl_goal_position[2] = {515, 215};  

  uint16_t dxl_present_position = 0;         

  // Open port
  if (openPort(port_num)) {
    printf("Succeeded to open the port!\n"); }

  // Set port baudrate
  if (setBaudRate(port_num, BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n"); }

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);

  // Write CW/CCW position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_CW_ANGLE_LIMIT, 215);
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_CCW_ANGLE_LIMIT, 515);


  // Write speed
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_MOVING_SPEED, 100);

 // Write goal position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index]);

    while(1){
    do
    {

      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_PRESENT_POSITION);
      //printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position[index], dxl_present_position);

      sleep(1);
    } while ((abs(dxl_goal_position[index] - dxl_present_position) > 10));

    // Change goal position
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
    }


  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);

  //Close port
  closePort(port_num);
}

void raise() {
  int port_num = portHandler(DEVICENAME);
  packetHandler();

  int index = 0;
  int dxl_goal_position[2] = {215, 515};  

  uint16_t dxl_present_position = 0;         

  // Open port
  if (openPort(port_num)) {
    printf("Succeeded to open the port!\n"); }

  // Set port baudrate
  if (setBaudRate(port_num, BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n"); }

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);

  // Write CW/CCW position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_CW_ANGLE_LIMIT, 215);
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_CCW_ANGLE_LIMIT, 515);


  // Write speed
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_MOVING_SPEED, 100);

 // Write goal position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index]);

    while(1){
    do
    {

      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_PRESENT_POSITION);
      //printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position[index], dxl_present_position);

      sleep(1);
    } while ((abs(dxl_goal_position[index] - dxl_present_position) > 10));

    // Change goal position
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
    }


  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);

  //Close port
  closePort(port_num);
}

void multi_turn() {
  int port_num = portHandler(DEVICENAME);
  packetHandler();

  int index = 0;
  int dxl_goal_position[2] = {0, 3000};  

  uint16_t dxl_present_position = 0;         

  // Open port
  if (openPort(port_num)) {
    printf("Succeeded to open the port!\n"); }

  // Set port baudrate
  if (setBaudRate(port_num, BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n"); }

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);

  // Write CW/CCW position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_CW_ANGLE_LIMIT, 0);
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_CCW_ANGLE_LIMIT, 0);


  // Write speed
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_MOVING_SPEED, 512);

 // Write goal position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index]);

    while(1){
    do
    {

      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_PRESENT_POSITION);
      //printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position[index], dxl_present_position);

      sleep(1);
    } while ((abs(dxl_goal_position[index] - dxl_present_position) > 10));

    // Change goal position
    if (index == 0)
    {
      index = 1;
    }
    else
    {
      index = 0;
    }
    }


  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);

  //Close port
  closePort(port_num);
}

void idle() {
  int port_num = portHandler(DEVICENAME);
  packetHandler();

  int index = 0;
  int dxl_goal_position[2] = {0, 3000};  

  uint16_t dxl_present_position = 0;         

  // Open port
  if (openPort(port_num)) {
    printf("Succeeded to open the port!\n"); }

  // Set port baudrate
  if (setBaudRate(port_num, BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n"); }

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);

  // Write CW/CCW position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_CW_ANGLE_LIMIT, 0);
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_CCW_ANGLE_LIMIT, 0);


  // Write speed
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_MOVING_SPEED, 0);

  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);

  //Close port
  closePort(port_num);
}
