#ifndef _TEENSY_H_
#define _TEENSY_H_

#include "./inout_interface/SPISlave_T4.h"
#include "./inout_interface/output_interface.h"
#include "./inout_interface/SPIInterface.h"
#include "./localization/localization.h"
#include "./path_follower/path_follower.h"
#include "./position_control/position_control.h"
#include "./regulator/regulator.h"
#include "utils.h"

typedef enum {
  ModeIdle, // No input from RPi, default is to remain still
  ModePositionControl,
  ModeSpeedControl
} controlmode_t; // Control modes type

// typedef struct TeensyController {
//     int control_time;
//     controlmode_t mode;
//     OutputInterface *outputs;
//     RobotPosition *robot_position;
//     PositionController *position_controller;
//     PathFollower *path_follower;
//     Regulator *speed_regulator;
// } TeensyController;

#endif