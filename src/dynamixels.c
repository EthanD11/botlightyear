#include "dynamixels.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <dynamixel_sdk.h>

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
#define DEVICENAME                      "/dev/ttyAMA0"      

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

int port_num;

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

void ping_dxl(int ID, float protocol) {
  //ping(port_num, protocol, ID);
  int dxl_comm_result = COMM_TX_FAIL;
  uint8_t dxl_error = 0;   
 
  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, protocol, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, protocol)) != COMM_SUCCESS)
  {
    printf("%s\n", getTxRxResult(protocol, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, protocol)) != 0)
  {
    printf("%s\n", getRxPacketError(protocol, dxl_error));
  }
  else
  {
    printf("Dynamixel has been successfully connected \n");
  }

  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, protocol, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
  
}

void deploy_solar_panel() {
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_goal_position = 515;

  uint8_t dxl_error = 0;    
  uint16_t dxl_present_position = 0; 

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
  if ((dxl_comm_result = getLastTxRxResult(port_num, AX_PROTOCOL_VERSION)) != COMM_SUCCESS) {
    printf("%s\n", getTxRxResult(AX_PROTOCOL_VERSION, dxl_comm_result));
  }
  else if ((dxl_error = getLastRxPacketError(port_num, AX_PROTOCOL_VERSION)) != 0) {
    printf("%s\n", getRxPacketError(AX_PROTOCOL_VERSION, dxl_error));
  }
  else {
    printf("Dynamixel has been successfully connected \n");
  }

  // Write CW/CCW position
  write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_CW_ANGLE_LIMIT, 215);
  write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_CCW_ANGLE_LIMIT, 515);

  // Write speed
  write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_MOVING_SPEED, 100);

  // Write goal position
  write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_GOAL_POSITION, dxl_goal_position);

    while ((abs(dxl_goal_position - dxl_present_position) > 10)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_AX_PRESENT_POSITION);
      write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("deployP [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position, dxl_present_position);
    }
    
  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}   
void raise_solar_panel() {
  //int dxl_comm_result = COMM_TX_FAIL;
  int dxl_goal_position = 215;  

  //uint8_t dxl_error = 0;    
  uint16_t dxl_present_position = 0;         

  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

  // Setup Dynamixel Torque
  write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_AX_TORQUE, 750);

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
  // Enable Dynamixel Torque
  write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

  // Write CW/CCW position
  write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_CW_ANGLE_LIMIT, 0);
  write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_CCW_ANGLE_LIMIT, 0);

  // Write speed
  while (read2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_AX_PRESENT_SPEED)<=0.8*ADDR_MOVING_SPEED){
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_MOVING_SPEED, 512);
  }
  sleep(3);

  // Write speed
  write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_MOVING_SPEED, 0);

  // Disable Dynamixel Torque
  write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}

void open_gripper() {
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_goal_position = 250;

    uint8_t dxl_error = 0;    
    uint16_t dxl_present_position = 0; 

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, XL_PROTOCOL_VERSION)) != COMM_SUCCESS) {
        printf("%s\n", getTxRxResult(XL_PROTOCOL_VERSION, dxl_comm_result));
    } else if ((dxl_error = getLastRxPacketError(port_num, XL_PROTOCOL_VERSION)) != 0) {
        printf("%s\n", getRxPacketError(XL_PROTOCOL_VERSION, dxl_error));
    } else {
        printf("Dynamixel has been successfully connected \n");
    }

    // Write Status return
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_STATUS_RETURN, 2); 

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
      write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 6, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("deployP [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position, dxl_present_position);
    }

    // Disable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}
void close_gripper() {
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_goal_position = 175;

    uint8_t dxl_error = 0;    
    uint16_t dxl_present_position = 0; 

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, 2.0)) != COMM_SUCCESS) {
        printf("%s\n", getTxRxResult(2.0, dxl_comm_result));
    } else if ((dxl_error = getLastRxPacketError(port_num, 2.0)) != 0) {
        printf("%s\n", getRxPacketError(2.0, dxl_error));
    } else {
        printf("Dynamixel has been successfully connected \n");
    }

    // Write Status return
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_STATUS_RETURN, 2); 

    // Write CW/CCW position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_CW_ANGLE_LIMIT, 175); 
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_CCW_ANGLE_LIMIT, 250);

    //Write Joint Mode
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_CONTROL_MODE, 2); 

    // Write speed
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_MOVING_SPEED, 100);

    // Write goal position
    write4ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_GOAL_POSITION, dxl_goal_position);

   while ((abs(dxl_goal_position - dxl_present_position) > 10)) {
      // Read present position
      dxl_present_position = read4ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_PRESENT_POSITION);
      write4ByteTxRx(port_num, XL_PROTOCOL_VERSION, 6, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("deployP [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position, dxl_present_position);
    }

    // Disable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}


void raiseG_withoutreturn() {
int dxl_comm_result = COMM_TX_FAIL;
    int dxl_goal_position = 138;

    uint8_t dxl_error = 0;    
    uint16_t dxl_present_position = 0; 

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, 2.0, 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, 2.0)) != COMM_SUCCESS) {
        printf("%s\n", getTxRxResult(2.0, dxl_comm_result));
    } else if ((dxl_error = getLastRxPacketError(port_num, 2.0)) != 0) {
        printf("%s\n", getRxPacketError(2.0, dxl_error));
    } else {
        printf("Dynamixel has been successfully connected \n");
    }

  // Write Status return
    write2ByteTxRx(port_num, 2.0, 1, 17, 2); 

    // Write CW/CCW position
    write2ByteTxRx(port_num, 2.0, 1, 6, 138); 
    write2ByteTxRx(port_num, 2.0, 1, 8, 456);

    //Write Joint Mode
    write2ByteTxRx(port_num, 2.0, 1, 11, 2); 

    // Write speed
    write2ByteTxRx(port_num, 2.0, 1, ADDR_MOVING_SPEED, 100);

    // Write goal position
    write2ByteTxRx(port_num, 2.0, 1, ADDR_GOAL_POSITION, dxl_goal_position);

    
        // Read present position
        dxl_present_position = read2ByteTxRx(port_num, 2.0, 1, 37);
      printf("abs : %i %i %i \n", dxl_goal_position, dxl_present_position, abs(dxl_goal_position - dxl_present_position));
      printf("t %i \n", read1ByteTxRx(port_num, 2.0, 1, 46));
        // Check if goal position is reached within a tolerance of 10
        /*if (abs(dxl_goal_position - dxl_present_position) <= 10) {
            break; // Exit loop if goal position reached
        }*/

        // Write goal position inside the loop to keep updating it
        write2ByteTxRx(port_num, 2.0, 1, ADDR_GOAL_POSITION, dxl_goal_position);
    printf("abs : %i %i %i \n", dxl_goal_position, dxl_present_position, abs(dxl_goal_position - dxl_present_position));


    // Disable Dynamixel Torque
    write1ByteTxRx(port_num, 2.0, 1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}
void deployG_withoutreturn() {
int dxl_comm_result = COMM_TX_FAIL;
    int dxl_goal_position = 456;

    uint8_t dxl_error = 0;    
    uint16_t dxl_present_position = 0; 

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, 2.0, 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, 2.0)) != COMM_SUCCESS) {
        printf("%s\n", getTxRxResult(2.0, dxl_comm_result));
    } else if ((dxl_error = getLastRxPacketError(port_num, 2.0)) != 0) {
        printf("%s\n", getRxPacketError(2.0, dxl_error));
    } else {
        printf("Dynamixel has been successfully connected \n");
    }

    // Write Status return
    write2ByteTxRx(port_num, 2.0, 1, 17, 2); 

    // Write CW/CCW position
    write2ByteTxRx(port_num, 2.0, 1, 6, 138); 
    write2ByteTxRx(port_num, 2.0, 1, 8, 456);

    //Write Joint Mode
    write2ByteTxRx(port_num, 2.0, 1, 11, 2); 

    // Write speed
    write2ByteTxRx(port_num, 2.0, 1, ADDR_MOVING_SPEED, 100);

    // Write goal position
    write2ByteTxRx(port_num, 2.0, 1, ADDR_GOAL_POSITION, dxl_goal_position);

    
        // Read present position
        dxl_present_position = read2ByteTxRx(port_num, 2.0, 1, 37);
      printf("abs : %i %i %i \n", dxl_goal_position, dxl_present_position, abs(dxl_goal_position - dxl_present_position));
      printf("t %i \n", read1ByteTxRx(port_num, 2.0, 1, 46));
        // Check if goal position is reached within a tolerance of 10
        /*if (abs(dxl_goal_position - dxl_present_position) <= 10) {
            break; // Exit loop if goal position reached
        }*/

        // Write goal position inside the loop to keep updating it
        write2ByteTxRx(port_num, 2.0, 1, ADDR_GOAL_POSITION, dxl_goal_position);
    printf("abs : %i %i %i \n", dxl_goal_position, dxl_present_position, abs(dxl_goal_position - dxl_present_position));


    // Disable Dynamixel Torque
    write1ByteTxRx(port_num, 2.0, 1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}

void openG() {
    int dxl_comm_result = COMM_TX_FAIL;
    int dxl_goal_position = 250;

    uint8_t dxl_error = 0;    
    uint16_t dxl_present_position = 0; 

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, 2.0, 3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, 2.0)) != COMM_SUCCESS) {
        printf("%s\n", getTxRxResult(2.0, dxl_comm_result));
    } else if ((dxl_error = getLastRxPacketError(port_num, 2.0)) != 0) {
        printf("%s\n", getRxPacketError(2.0, dxl_error));
    } else {
        printf("Dynamixel has been successfully connected \n");
    }

    // Write CW/CCW position
    write2ByteTxRx(port_num, 2.0, 3, 6, 175); 
    write2ByteTxRx(port_num, 2.0, 3, 8, 250);

    //Write Joint Mode
    write2ByteTxRx(port_num, 2.0, 3, 11, 2); 

    // Write speed
    write2ByteTxRx(port_num, 2.0, 3, ADDR_MOVING_SPEED, 100);

    // Write goal position
    write2ByteTxRx(port_num, 2.0, 3, ADDR_GOAL_POSITION, dxl_goal_position);

    while (abs(dxl_goal_position - dxl_present_position) > 10) {
        // Read present position
        dxl_present_position = read2ByteTxRx(port_num, 2.0, 3, 37);
      printf("abs : %i %i %i \n", dxl_goal_position, dxl_present_position, abs(dxl_goal_position - dxl_present_position));
      printf("t %i \n", read1ByteTxRx(port_num, 2.0, 3, 46));
        // Check if goal position is reached within a tolerance of 10
        /*if (abs(dxl_goal_position - dxl_present_position) <= 10) {
            break; // Exit loop if goal position reached
        }*/

        // Write goal position inside the loop to keep updating it
        write2ByteTxRx(port_num, 2.0, 3, ADDR_GOAL_POSITION, dxl_goal_position);
    }
    printf("abs : %i %i %i \n", dxl_goal_position, dxl_present_position, abs(dxl_goal_position - dxl_present_position));


    // Disable Dynamixel Torque
    write1ByteTxRx(port_num, 2.0, 3, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
}  

void close_port() {
  closePort(port_num);
}