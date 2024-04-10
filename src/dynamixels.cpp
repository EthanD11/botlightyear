#include "Dynamixels_sdk/dynamixel_sdk.h"
#include "dynamixels.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

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

// Protocol version
#define AX_PROTOCOL_VERSION             1.0      
#define XL_PROTOCOL_VERSION             2.0           

// Default setting
#define BAUDRATE                        57600
#define DEVICENAME                      "/dev/ttyUSB0"      

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold
#define DTHETA                          5                   // Delta theta on angles

int port_num; 

void dxl_init_port() {
    port_num = portHandler(DEVICENAME);
    packetHandler();

    if (!openPort(port_num)) {
        printf("Failed to open the port!\n");
        exit(1);
    }

    if (!setBaudRate(port_num, BAUDRATE)) {
        printf("Failed to set the baudrate!\n");
        exit(1);
    }   
}

void dxl_close_port() {
    closePort(port_num);
}   

void dxl_ping(int ID, float PROTOCOL) {
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;   
 
    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL, ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL)) != COMM_SUCCESS)
    {
        printf("%s\n", getTxRxResult(PROTOCOL, dxl_comm_result));
        exit(1);
    }
    if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL)) != 0)
    {
        printf("%s\n", getRxPacketError(PROTOCOL, dxl_error));
        exit(1);
    }
    printf("Dynamixel %03d has been successfully connected \n", ID);
}

void dxl_idle(int ID, float PROTOCOL) { 
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0;  

    //Disable Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL, ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL)) != COMM_SUCCESS)
    {
        printf("%s\n", getTxRxResult(PROTOCOL, dxl_comm_result));
        exit(1);
    }
    if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL)) != 0)
    {
        printf("%s\n", getRxPacketError(PROTOCOL, dxl_error));
        exit(1);
    }
    printf("Dynamixel %03d has been successfully idled \n", ID);
}

void dxl_deploy(position_t position) {
    uint16_t dxl_goal_position = 0;
    uint16_t dxl_present_position = 0;

    switch(position) {
        case Up: 
            dxl_goal_position = 215;
            break;
        case Down:
            dxl_goal_position = 520;
            break;
        case Mid:
            dxl_goal_position = 365;
            break;
    }

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    // Write CW/CCW position
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_CW_ANGLE_LIMIT, 215); 
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_CCW_ANGLE_LIMIT, 520);

    // Write speed
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_MOVING_SPEED, 200);
    
    // Write goal position
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_GOAL_POSITION, dxl_goal_position);

    while ((abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_AX_PRESENT_POSITION);
      write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("position_solar [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position, dxl_present_position);
    }
}

void dxl_multiturn(direction_t direction) {
    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    // Write CW/CCW position
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_CW_ANGLE_LIMIT, 0); 
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_CCW_ANGLE_LIMIT, 0);

    switch(direction) {
        case CCW: 
            // Write speed
            while (read2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_AX_PRESENT_SPEED)<=0.8*ADDR_MOVING_SPEED){
            write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_MOVING_SPEED, 1024+512);
            }
            break;
        case CW:
            // Write speed
            while (read2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_AX_PRESENT_SPEED)<=0.8*ADDR_MOVING_SPEED){
            write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_MOVING_SPEED, 512);
            }
            break;
    }
    sleep(2);

    // Write speed
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_MOVING_SPEED, 0);
}

void dxl_position(double goal_pos) {
    uint16_t dxl_goal_position = goal_pos;
    uint16_t dxl_present_position = 0;
    dxl_present_position = read2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_AX_PRESENT_POSITION);

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    // Write CW/CCW position
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_CW_ANGLE_LIMIT, 0); 
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_CCW_ANGLE_LIMIT, 1023);

    // Write speed
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_MOVING_SPEED, 200);

    // Write goal position
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_GOAL_POSITION, dxl_goal_position);

   while ((abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_AX_PRESENT_POSITION);
      write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("position_solar [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position, dxl_present_position);
    }
}

void dxl_turn(team_t team, double angle) {
    uint16_t dxl_goal_position = 512;
    uint16_t dxl_present_position = dxl_present_position = read2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_AX_PRESENT_POSITION);

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    // Write CW/CCW position
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_CW_ANGLE_LIMIT, 0); 
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_CCW_ANGLE_LIMIT, 1023);

    // Write speed
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_MOVING_SPEED, 200);

    switch (team) {
        case Blue: 
            if (angle == 0) {
                dxl_goal_position = 750; 
            }
            else if ((angle >= -180) && (angle < -90)) {
                dxl_goal_position = (int)(2.84*angle + 740);
            }
            else if ((angle >= -90) && (angle < 0)) {
                dxl_goal_position = (int)(4.3*angle + 903);
            }
            else if ((angle > 0) && (angle <= 45)) {
                dxl_goal_position = (int)(2.65*angle + 893);
            }
            else if ((angle > 45) && (angle <= 90)) {
                dxl_goal_position = 0;
            }
            else if ((angle >= 90) && (angle < 150)) {
                dxl_goal_position = (int)(3.24*angle -288);
            }
            else if (angle >= 150) {
                dxl_goal_position = 200;
            }
            else {
                printf("Undefined angle, Blue"); 
            }
         break;

        case Yellow:
            if (angle == 0) {
                dxl_goal_position = 170; 
            }
            else if ((angle >= -180) && (angle <=-90)) {
                dxl_goal_position = (int)(1.7*angle + 1100); 
            }
            else if ((angle > -90) && (angle <= -70)) {
                dxl_goal_position = (int)(3.65*angle + 1275); 
            }
            else if ((angle >= -70) && (angle <= -45)) {
                dxl_goal_position = 1023; 
            }
            else if ((angle > -45) && (angle < 0)) {
                dxl_goal_position = (int)(3.8*angle + 167); 
            }
            else if ((angle > 0) && (angle < 90)) {
                dxl_goal_position = (int)(2.7*angle + 150); 
            }
            else if (angle >= 90) {
                dxl_goal_position = (int)(2.2*angle + 323); 
            }
            else {
                printf("Undefined angle, Yellow\n"); 
            }
            break;
    }

    // Write goal position
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_GOAL_POSITION, dxl_goal_position);

   while ((abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_AX_PRESENT_POSITION);
      write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("position_solar [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position, dxl_present_position);
    }
}

void dxl_init_sp() { 
    uint16_t dxl_goal_position = 512;
    uint16_t dxl_present_position = 0;

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    // Write CW/CCW position
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_CW_ANGLE_LIMIT, 0); 
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_CCW_ANGLE_LIMIT, 1023);

    // Write speed
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_MOVING_SPEED, 200);
    
    // Write goal position
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_GOAL_POSITION, dxl_goal_position);

   while ((abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_AX_PRESENT_POSITION);
      write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("position_solar [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 6, dxl_goal_position, dxl_present_position);
    }
}

void solar_panel(team_t team, double angle) {
    dxl_deploy(Down); 
    dxl_turn(team, angle);
    dxl_deploy(Up); 
    dxl_init_sp(); 
}

