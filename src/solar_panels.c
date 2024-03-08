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
#define ADDR_MX_TORQUE                  14
#define ADDR_PRESENT_SPEED              38

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting

#define DXL_ID                          6                  // Dynamixel ID
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyAMA0"      

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

int port_num;
#define ESC_ASCII_VALUE                 0x1b

void init_port() {
  port_num = portHandler(DEVICENAME);
  packetHandler();

  // Open port
  if (openPort(port_num)) {
    printf("Succeeded to open the port!\n"); }

  // Set port baudrate
  if (setBaudRate(port_num, BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n"); }
}

void deployP() {
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_goal_position= 515;

  uint8_t dxl_error = 0;    
  uint16_t dxl_present_position = 0; 

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }

  // Write CW/CCW position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_CW_ANGLE_LIMIT, 215);
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_CCW_ANGLE_LIMIT, 515);

  // Write speed
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_MOVING_SPEED, 100);

 // Write goal position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position);

    while ((abs(dxl_goal_position - dxl_present_position) > 10)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_PRESENT_POSITION);
      write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position);
      //printf("deployP [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position, dxl_present_position);
      }
    
  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
}   
void deploy() {
  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_goal_position[2] = {515, 215};

  uint8_t dxl_error = 0;    
  uint16_t dxl_present_position = 0; 

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
  {
    printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }

  // Write CW/CCW position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_CW_ANGLE_LIMIT, 215);
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_CCW_ANGLE_LIMIT, 515);

  // Write speed
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_MOVING_SPEED, 100);

 // Write goal position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index]);

    while ((abs(dxl_goal_position[index] - dxl_present_position) > 10)) {

      for (uint16_t i = 0; i < 50; i++)
      {
        uint8_t res = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, i);
        /*if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
          printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        }
        else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
        {
          printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
        }
        printf("%d : %d\n", i, res);*/
      }

      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_PRESENT_POSITION);
      if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
      {
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
      }
      else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
      {
        printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
      }
      printf("deploy [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position[index], dxl_present_position);

    

    // Chae goal position
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

}

void raiseP() {
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_goal_position = 215;  

  uint8_t dxl_error = 0;    
  uint16_t dxl_present_position = 0;         

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);

  // Setup Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE, 750);

  // Write CW/CCW position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_CW_ANGLE_LIMIT, 215);
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_CCW_ANGLE_LIMIT, 515);

  // Write speed
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_MOVING_SPEED, 100);

 // Write goal position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position);

    while ((abs(dxl_goal_position - dxl_present_position) > 10)) {
      write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position);
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_PRESENT_POSITION);
      //printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position[index], dxl_present_position);
    }

  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);

}
void raise() {

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_goal_position[2] = {215, 515};  

  uint8_t dxl_error = 0;    
  uint16_t dxl_present_position = 0;         

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);

  // Setup Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_TORQUE, 750);

  // Write CW/CCW position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_CW_ANGLE_LIMIT, 215);
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_CCW_ANGLE_LIMIT, 515);

  // Write speed
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_MOVING_SPEED, 100);

 // Write goal position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index]);

    while ((abs(dxl_goal_position[index] - dxl_present_position) > 10)) {

     for (uint16_t i = 0; i < 50; i++)
      {
        uint8_t res = read1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, i);
        if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
          printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
        }
        else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
        {
          printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
        }
        printf("%d : %d\n", i, res);
      }

      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_PRESENT_POSITION);
      //printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position[index], dxl_present_position);

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

}

void multi_turnP() {
  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);

  // Write CW/CCW position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_CW_ANGLE_LIMIT, 0);
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_CCW_ANGLE_LIMIT, 0);

  // Write speed
  while (read2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_PRESENT_SPEED)<=0.8*ADDR_MX_MOVING_SPEED){
    write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_MOVING_SPEED, 512);
  }
  sleep(3);

  // Write speed
  
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_MOVING_SPEED, 0);

  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
}
void multi_turn() {
  int index = 0;
  int dxl_goal_position[2] = {0, 3000};  

  uint16_t dxl_present_position = 0;         

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);

  // Write CW/CCW position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_CW_ANGLE_LIMIT, 0);
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_CCW_ANGLE_LIMIT, 0);

  // Write speed
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_MOVING_SPEED, 512);

 // Write goal position
 /*
  write2ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index]);

    while ((abs(dxl_goal_position[index] - dxl_present_position) > 10)) {

      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, PROTOCOL_VERSION, 6, ADDR_MX_PRESENT_POSITION);
      //printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position[index], dxl_present_position);

    

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
*/ 

sleep(1);

// Write speed
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_MOVING_SPEED, 0);

  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
}

void idle() {
  int index = 0;
  int dxl_goal_position[2] = {0, 3000};  

  uint16_t dxl_present_position = 0;         

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);

  // Write CW/CCW position
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_CW_ANGLE_LIMIT, 0);
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_CCW_ANGLE_LIMIT, 0);

  // Write speed
  write2ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_MOVING_SPEED, 0);

  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, PROTOCOL_VERSION, 8, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
}

void close_port() {
  closePort(port_num);
}