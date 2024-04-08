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

// Control table address for Dynamixel XL-320
#define ADDR_XL_CONTROL_MODE            11                 // 1: WHEEL MODE, 2: JOINT MODE
#define ADDR_XL_TORQUE                  15
#define ADDR_XL_STATUS_RETURN           17
#define ADDR_XL_PRESENT_POSITION        37
#define ADDR_XL_PRESENT_SPEED           39
#define ADDR_XL_REGISTERED              47
#define ADDR_XL_MOVING_STATUS           49
#define ADDR_XL_PUNCH                   51

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

void position_solar(sp_position_t position) {
    uint16_t dxl_goal_position = 0;
    uint16_t dxl_present_position = 0;

    switch(position) {
        case UpS: 
            dxl_goal_position = 215;
            break;
        case DownS:
            dxl_goal_position = 520;
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

void multiturn_solar(direction_t direction) {
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

void turn_solar(team_t team, double pres_angle) {
    uint16_t dxl_goal_position = 512;
    uint16_t dxl_present_position = 0;
    dxl_present_position = read2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_AX_PRESENT_POSITION);

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, AX_PROTOCOL_VERSION, 8, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    // Write CW/CCW position
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_CW_ANGLE_LIMIT, 0); 
    write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_CCW_ANGLE_LIMIT, 1023);

    switch (team) {
        case Blue: 
        // Write speed
        write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_MOVING_SPEED, 200);
        if (abs(pres_angle + 0) < DTHETA) {
            dxl_goal_position = 780; 
        }
        else if (-90 < pres_angle < 0) {
            dxl_goal_position = (134/45)*pres_angle + 780;
        }
        else if (abs(pres_angle + 90) < DTHETA) {
            dxl_goal_position = 512; //No move
        }
        else if (-180 < pres_angle < -90) {
            dxl_goal_position = (181/45)*pres_angle + 874;
        }
        else if (abs(pres_angle + 180) < DTHETA) {
            dxl_goal_position = 150; 
        }
        else if (pres_angle > 90) { //Opposite team
            dxl_goal_position = (-5/3)*pres_angle + 212;
        }
        else if (abs(pres_angle - 90) < DTHETA) { //Opposite team
            dxl_goal_position = 0;
        }
        else {
            printf("Undefined angle, Blue\n"); 
        }
        break;

        case Yellow:
        // Write speed
        write2ByteTxRx(port_num, AX_PROTOCOL_VERSION, 6, ADDR_MOVING_SPEED, 200);
        if (abs(pres_angle - 0) < DTHETA) {
            dxl_goal_position = 150; 
        }
        else if (0 < pres_angle < 90) {
            dxl_goal_position = (181/45)*pres_angle + 150;
        }
        else if (abs(pres_angle - 90) < DTHETA) {
            dxl_goal_position = 512; //No move
        }
        else if (90 < pres_angle < 180) {
            dxl_goal_position = (134/45)*pres_angle + 244;
        }
        else if (abs(pres_angle - 180) < DTHETA) {
            dxl_goal_position = 780; 
        }
        else if (pres_angle < -90) {
            dxl_goal_position = (27/10)*pres_angle + 26;
        }
        else if (abs(pres_angle + 90) < DTHETA) { //Opposite team
            dxl_goal_position = 1023;
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

void init_sp() { 
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
   if ((team == Blue) and ((-30 < angle) or (angle < -135))) {
    position_solar(DownS); 
    turn_solar(team, angle);
    position_solar(UpS); 
    } 

   else if ((team == Yellow) and (30 < angle < 130)) {
    position_solar(DownS); 
    turn_solar(team, angle);
    position_solar(UpS); 
    }

    init_sp(); 
}


void gripper(object_t object) {
    uint16_t dxl_goal_position = 0;
    uint16_t dxl_present_position = 0;

    switch(object) {
        case Open: 
            dxl_goal_position = 275;
            break;
        case Close:
            dxl_goal_position = 185;
            break;
        case Plant:
            dxl_goal_position = 220;
            break;
        case Pot:
            dxl_goal_position = 230;
            break;
    }

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    // Write CW/CCW position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_CW_ANGLE_LIMIT, 160); 
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_CCW_ANGLE_LIMIT, 290);

    //Write Joint Mode
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_CONTROL_MODE, 2); 

    // Write speed
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_MOVING_SPEED, 200);

    // Write punch
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_PUNCH, 80);

    //Write Max Torque
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_TORQUE, 1023);
    
    // Write goal position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_GOAL_POSITION, dxl_goal_position);

   while ((abs(dxl_goal_position - dxl_present_position) > 15)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_XL_PRESENT_POSITION);
      write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 3, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("open_gripper [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 3, dxl_goal_position, dxl_present_position);
    }
}

void position_gripper(position_t position) {
    uint16_t dxl_goal_position = 0;
    uint16_t dxl_present_position = 0;

    switch(position) {
        case Up: 
            dxl_goal_position = 210;
            break;
        case Down:
            dxl_goal_position = 440;
            break;
        case MidPlant:
            dxl_goal_position = 350;
            break;
        case MidPot:
            dxl_goal_position = 350;
            break;
    }

    // Enable Dynamixel Torque
    write1ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);

    // Write CW/CCW position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_CW_ANGLE_LIMIT, 135); 
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_CCW_ANGLE_LIMIT, 460);

    //Write Joint Mode
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_XL_CONTROL_MODE, 2); 

    // Write speed
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_MOVING_SPEED, 200);
    
    // Write goal position
    write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_GOAL_POSITION, dxl_goal_position);

   while ((abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD)) {
      // Read present position
      dxl_present_position = read2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_XL_PRESENT_POSITION);
      write2ByteTxRx(port_num, XL_PROTOCOL_VERSION, 1, ADDR_GOAL_POSITION, dxl_goal_position);
      //printf("position_gripper [ID:%03d] GoalPos:%03d  PresPos:%03d\n", 1, dxl_goal_position, dxl_present_position);
    }
}