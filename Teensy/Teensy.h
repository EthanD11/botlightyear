#ifndef _TEENSY_H_
#define _TEENSY_H_

// src subfolder name is COMPULSORY
// see: https://forum.arduino.cc/t/subfolders-in-sketch-folder/564852/4

#include "src/inout_interface/SPISlave_T4.h"
#include "src/inout_interface/output_interface.h"
#include "src/inout_interface/SPIInterface.h"
#include "src/localization/localization.h"
#include "src/path_follower/path_follower.h"
#include "src/position_control/position_control.h"
#include "src/regulator/regulator.h"
#include "utils.h"

typedef struct TeensyController {
    int control_time;
} TeensyController;

#endif