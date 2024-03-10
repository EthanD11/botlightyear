#include "dynamixels.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "../Dynamixels/include/dynamixel_sdk/dynamixel_sdk.h"

// Control table address for both Dynamixel AX-12 and XL-320
#define ADDR_CW_ANGLE_LIMIT             6                  // JOINT MODE: neither are 0
#define ADDR_CCW_ANGLE_LIMIT            8                  // WHEEL MODE: both are 0
#define ADDR_TORQUE_ENABLE              24                  
#define ADDR_GOAL_POSITION              30
#define ADDR_MOVING_SPEED               32

// Control table address for Dynamixel AX-12
#define ADDR_AX_TORQUE                  14
#define ADDR_AX_STATUS_RETURN           16
#define ADDR_AX_PRESENT_POSITION        36
#define ADDR_AX_PRESENT_SPEED           38
#define ADDR_AX_REGISTERED              44
#define ADDR_AX_MOVING_STATUS           46

// Control table address for Dynamixel XL-320
#define ADDR_XL_CONTROL_MODE            11                 // 1: WHEEL MODE, 2: JOINT MODE
#define ADDR_XL_TORQUE                  15
#define ADDR_XL_STATUS_RETURN           17
#define ADDR_XL_PRESENT_POSITION        37
#define ADDR_XL_PRESENT_SPEED           39
#define ADDR_XL_REGISTERED              47
#define ADDR_XL_MOVING_STATUS           49

// Protocol version
#define AX_PROTOCOL_VERSION             1.0      
#define XL_PROTOCOL_VERSION             2.0           

// Default setting
#define BAUDRATE                        57600
#define XL_DEVICENAME                   "/dev/ttyUSB0"      
#define AX_DEVICENAME                   "/dev/ttyUSB0" 

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

int xl_port_num;
int ax_port_num;

void ax_init_port() {
  ax_port_num = portHandler(AX_DEVICENAME);
  packetHandler();

  // Open port
  if (openPort(ax_port_num)) {
    printf("Succeeded to open the port!\n"); }

  // Set port baudrate
  if (setBaudRate(ax_port_num, BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n"); }
}
void xl_init_port() {
  xl_port_num = portHandler(XL_DEVICENAME);
  packetHandler();

  // Open port
  if (openPort(xl_port_num)) {
    printf("Succeeded to open the port!\n"); }

  // Set port baudrate
  if (setBaudRate(xl_port_num, BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n"); }
}

void ax_ping(int ID) {
  int protocol = AX_PROTOCOL_VERSION;
  //ping(port_num, protocol, ID);
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;   
 
  // Enable Dynamixel Torque
  write1ByteTxRx(ax_port_num, protocol, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(ax_port_num, protocol)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(protocol, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(ax_port_num, protocol)) != 0)
  {
    printf("%s\n", getRxPacketError(protocol, dxl_error));
  }
  else
  {
    printf("Dynamixel %03d has been successfully connected \n", ID);
  }

  // Disable Dynamixel Torque
  write1ByteTxRx(ax_port_num, protocol, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}
void xl_ping(int ID) {
  int protocol = XL_PROTOCOL_VERSION;
  //ping(port_num, protocol, ID);
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;   
 
  // Enable Dynamixel Torque
  write1ByteTxRx(xl_port_num, protocol, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(xl_port_num, protocol)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(protocol, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(xl_port_num, protocol)) != 0)
  {
    printf("%s\n", getRxPacketError(protocol, dxl_error));
  }
  else
  {
    printf("Dynamixel %03d has been successfully connected \n", ID);
  }

  // Disable Dynamixel Torque
  write1ByteTxRx(xl_port_num, protocol, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}

void deploy_solar_panel() {
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_goal_position = 515;

  uint8_t dxl_error = 0;    
  uint16_t dxl_present_position = 0; 

  // Enable Dynamixel Torque
  write1ByteTxRx(ax_port_num, AX_PROTOCOL_VERSION, 6, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

  // Write CW/CCW position
  write2ByteTxRx(ax_port_num, AX_PROTOCOL_VERSION, 6, ADDR_CW_ANGLE_LIMIT, 215);
  write2ByteTxRx(ax_port_num, AX_PROTOCOL_VERSION, 6, ADDR_CCW_ANGLE_LIMIT, 515);

  // Write speed
  write2ByteTxRx(ax_port_num, AX_PROTOCOL_VERSION, 6, ADDR_MOVING_SPEED, 100);

  // Write goal position
  write2ByteTxRx(ax_port_num, AX_PROTOCOL_VERSION, 6, ADDR_GOAL_POSITION, dxl_goal_position);

    while ((abs(dxl_goal_position - dxl_present_position) > 10)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(ax_port_num, AX_PROTOCOL_VERSION, 6, ADDR_AX_PRESENT_POSITION);
      write2ByteTxRx(ax_port_num, AX_PROTOCOL_VERSION, 6, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("deployP [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position, dxl_present_position);
    }
    
  // Disable Dynamixel Torque
  write1ByteTxRx(ax_port_num, AX_PROTOCOL_VERSION, 6, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}   
void raise_solar_panel() {
  int port_num = ax_port_num; 
  //int dxl_comm_result = COMM_TX_FAIL;
  int dxl_goal_position = 215;  

  //uint8_t dxl_error = 0;    
  uint16_t dxl_present_position = 0;         

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

  // Setup Dynamixel Torque
  uint32_t max_torque = (uint32_t)750;
  write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_AX_TORQUE, max_torque);

  // Write CW/CCW position
  write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_CW_ANGLE_LIMIT, 215);
  write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_CCW_ANGLE_LIMIT, 515);

  // Write speed
  write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_MOVING_SPEED, 100);

 // Write goal position
  write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_GOAL_POSITION, dxl_goal_position);

    while ((abs(dxl_goal_position - dxl_present_position) > 10)) {
      write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_GOAL_POSITION, dxl_goal_position);
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_AX_PRESENT_POSITION);
      //printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position[index], dxl_present_position);
    }

  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);

}
void multi_turn_solar_panel() {
  int port_num = ax_port_num;
  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

  // Write CW/CCW position
  write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_CW_ANGLE_LIMIT, 0);
  write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_CCW_ANGLE_LIMIT, 0);

  // Write speed
  while (read2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_AX_PRESENT_SPEED)<=0.8*ADDR_MOVING_SPEED){
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_MOVING_SPEED, 512);
  }
  sleep(1.5);

  // Write speed
  write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_MOVING_SPEED, 0);

  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}

void open_gripper() {
    int port_num = xl_port_num;
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_goal_position = 250;

    uint8_t dxl_error = 0;    
    uint16_t dxl_present_position = 0; 

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    // Write Status return
    //write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_STATUS_RETURN, 2); 

    // Write CW/CCW position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_CW_ANGLE_LIMIT, 175); 
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_CCW_ANGLE_LIMIT, 250);

    //Write Joint Mode
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_CONTROL_MODE, 2); 

    // Write speed
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_MOVING_SPEED, 100);

    // Write goal position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_GOAL_POSITION, dxl_goal_position);

    /*
        // Read present position
        dxl_present_position = read2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_PRESENT_POSITION);
      printf("abs : %i %i %i \n", dxl_goal_position, dxl_present_position, abs(dxl_goal_position - dxl_present_position));
      //printf("t %i \n", read1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, 46));
        // Check if goal position is reached within a tolerance of 10
        if (abs(dxl_goal_position - dxl_present_position) <= 10) {
            break; // Exit loop if goal position reached
        }

        // Write goal position inside the loop to keep updating it
        write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_GOAL_POSITION, dxl_goal_position);
    printf("abs : %i %i %i \n", dxl_goal_position, dxl_present_position, abs(dxl_goal_position - dxl_present_position));
    */

   while ((abs(dxl_goal_position - dxl_present_position) > 10)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_PRESENT_POSITION);
      write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("open_gripper [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 3, dxl_goal_position, dxl_present_position);
      
      /*for (uint16_t i = 0; i < 50; i++)
      {
        uint8_t res = read1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, i);
        /*if ((dxl_comm_result = getLastTxRxResult(port_num, XL_PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
          printf("%s\n", getTxRxResult(XL_PROTOCOL_VERSION, dxl_comm_result));
        }
        else if ((dxl_error = getLastRxPacketError(port_num, XL_PROTOCOL_VERSION)) != 0)
        {
          printf("%s\n", getRxPacketError(XL_ROTOCOL_VERSION, dxl_error));
        }
        printf("%d : %d\n", i, res);
      }*/
    }

    // Disable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}
void close_gripper() {
    int port_num = xl_port_num;
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_goal_position = 175;

    uint8_t dxl_error = 0;    
    uint16_t dxl_present_position = 0; 

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    // Write Status return
    //write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_STATUS_RETURN, 2); 

    // Write CW/CCW position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_CW_ANGLE_LIMIT, 175); 
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_CCW_ANGLE_LIMIT, 250);

    //Write Joint Mode
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_CONTROL_MODE, 2); 

    // Write speed
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_MOVING_SPEED, 100);

    // Write goal position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_GOAL_POSITION, dxl_goal_position);

   while ((abs(dxl_goal_position - dxl_present_position) > 10)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_PRESENT_POSITION);
      write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("close_gripper [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 3, dxl_goal_position, dxl_present_position);
      /*for (uint16_t i = 0; i < 50; i++)
      {
        uint8_t res = read1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, i);
        /*if ((dxl_comm_result = getLastTxRxResult(port_num, XL_PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
          printf("%s\n", getTxRxResult(XL_PROTOCOL_VERSION, dxl_comm_result));
        }
        else if ((dxl_error = getLastRxPacketError(port_num, XL_PROTOCOL_VERSION)) != 0)
        {
          printf("%s\n", getRxPacketError(XL_ROTOCOL_VERSION, dxl_error));
        }
        printf("%d : %d\n", i, res);
      }*/
    }

    // Disable Dynamixel Torque
    //write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}
void close_gripper_plant() {
    int port_num = xl_port_num;
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_goal_position = 200;

    uint8_t dxl_error = 0;    
    uint16_t dxl_present_position = 0; 

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    // Write Status return
    //write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_STATUS_RETURN, 2); 

    // Write CW/CCW position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_CW_ANGLE_LIMIT, 175); 
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_CCW_ANGLE_LIMIT, 250);

    //Write Joint Mode
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_CONTROL_MODE, 2); 

    // Write speed
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_MOVING_SPEED, 100);

    // Write goal position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_GOAL_POSITION, dxl_goal_position);

   while ((abs(dxl_goal_position - dxl_present_position) > 20)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_PRESENT_POSITION);
      write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("close_gripper [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 3, dxl_goal_position, dxl_present_position);
      /*for (uint16_t i = 0; i < 50; i++)
      {
        uint8_t res = read1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, i);
        /*if ((dxl_comm_result = getLastTxRxResult(port_num, XL_PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
          printf("%s\n", getTxRxResult(XL_PROTOCOL_VERSION, dxl_comm_result));
        }
        else if ((dxl_error = getLastRxPacketError(port_num, XL_PROTOCOL_VERSION)) != 0)
        {
          printf("%s\n", getRxPacketError(XL_ROTOCOL_VERSION, dxl_error));
        }
        printf("%d : %d\n", i, res);
      }*/
    }

    // Disable Dynamixel Torque
    //write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}

void raise_gripper() {
    int port_num = xl_port_num;
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_goal_position = 138;

    uint8_t dxl_error = 0;    
    uint16_t dxl_present_position = 0; 

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    // Write Status return
    //write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_STATUS_RETURN, 2); 

    // Write CW/CCW position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_CW_ANGLE_LIMIT, 138); 
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_CCW_ANGLE_LIMIT, 456);

    //Write Joint Mode
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_XL_CONTROL_MODE, 2); 

    // Write speed
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_MOVING_SPEED, 100);

    // Write goal position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_GOAL_POSITION, dxl_goal_position);

    /*
        // Read present position
        dxl_present_position = read2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_PRESENT_POSITION);
      printf("abs : %i %i %i \n", dxl_goal_position, dxl_present_position, abs(dxl_goal_position - dxl_present_position));
      //printf("t %i \n", read1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, 46));
        // Check if goal position is reached within a tolerance of 10
        if (abs(dxl_goal_position - dxl_present_position) <= 10) {
            break; // Exit loop if goal position reached
        }

        // Write goal position inside the loop to keep updating it
        write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_GOAL_POSITION, dxl_goal_position);
    printf("abs : %i %i %i \n", dxl_goal_position, dxl_present_position, abs(dxl_goal_position - dxl_present_position));
    */

   while ((abs(dxl_goal_position - dxl_present_position) > 20)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_XL_PRESENT_POSITION);
      write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("open_gripper [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 1, dxl_goal_position, dxl_present_position);
      
      /*for (uint16_t i = 0; i < 50; i++)
      {
        uint8_t res = read1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, i);
        /*if ((dxl_comm_result = getLastTxRxResult(port_num, XL_PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
          printf("%s\n", getTxRxResult(XL_PROTOCOL_VERSION, dxl_comm_result));
        }
        else if ((dxl_error = getLastRxPacketError(port_num, XL_PROTOCOL_VERSION)) != 0)
        {
          printf("%s\n", getRxPacketError(XL_ROTOCOL_VERSION, dxl_error));
        }
        printf("%d : %d\n", i, res);
      }*/
    }

    // Disable Dynamixel Torque
    //write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}
void deploy_gripper() {
    int port_num = xl_port_num;
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_goal_position = 456;

    uint8_t dxl_error = 0;    
    uint16_t dxl_present_position = 0; 

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    // Write Status return
    //write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_STATUS_RETURN, 2); 

    // Write CW/CCW position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_CW_ANGLE_LIMIT, 138); 
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_CCW_ANGLE_LIMIT, 456);

    //Write Joint Mode
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_XL_CONTROL_MODE, 2); 

    // Write speed
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_MOVING_SPEED, 200);

    // Write goal position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_GOAL_POSITION, dxl_goal_position);

   while ((abs(dxl_goal_position - dxl_present_position) > 20)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_XL_PRESENT_POSITION);
      write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("close_gripper [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 31, dxl_goal_position, dxl_present_position);
      /*for (uint16_t i = 0; i < 50; i++)
      {
        uint8_t res = read1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, i);
        /*if ((dxl_comm_result = getLastTxRxResult(port_num, XL_PROTOCOL_VERSION)) != COMM_SUCCESS)
        {
          printf("%s\n", getTxRxResult(XL_PROTOCOL_VERSION, dxl_comm_result));
        }
        else if ((dxl_error = getLastRxPacketError(port_num, XL_PROTOCOL_VERSION)) != 0)
        {
          printf("%s\n", getRxPacketError(XL_ROTOCOL_VERSION, dxl_error));
        }
        printf("%d : %d\n", i, res);
      }*/
    }

    // Disable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}
void mid_gripper() {
    int port_num = xl_port_num;
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_goal_position = 275;

    uint8_t dxl_error = 0;    
    uint16_t dxl_present_position = 0; 

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    // Write Status return
    //write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_STATUS_RETURN, 2); 

    // Write CW/CCW position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_CW_ANGLE_LIMIT, 138); 
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_CCW_ANGLE_LIMIT, 456);

    //Write Joint Mode
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_XL_CONTROL_MODE, 2); 

    // Write speed
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_MOVING_SPEED, 100);

    // Write goal position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_GOAL_POSITION, dxl_goal_position);

   while ((abs(dxl_goal_position - dxl_present_position) > 20)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_XL_PRESENT_POSITION);
      write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_GOAL_POSITION, dxl_goal_position);
      
    }

    // Disable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}

void ax_close_port() {
  closePort(ax_port_num);
}
void xl_close_port() {
  closePort(xl_port_num);
}
