#include "src/inout_interface/output_interface.h"
#include "src/inout_interface/SPIInterface.h"
#include "src/localization/localization.h"
#include "src/path_follower/path_follower.h"
#include "src/position_control/position_control.h"
#include "src/regulator/regulator.h"


#ifndef _SHARED_VARIABLE_
#define _SHARED_VARIABLE_

typedef enum _controlmode_t: int8_t 
{
	ModeIdle, // No input from RPi, default is to remain still
	ModePositionControl,
	ModePathFollowingInit,
	ModePathFollowing,
	ModeSpeedControl,
	ModeConstantDC,
	ModePositionControlOver
} controlmode_t; // Control modes type
        // Note : Left = 1, Right = 2

class SharedVariables {
    public:
        // ----- GENERALS -----
        // Current and reference x, y and theta
        OutputInterface *outputs;
        RobotPosition *robot_position;
        PositionController *position_controller;
        PathFollower *path_follower;
        Regulator *speed_regulator;
        // double spi_speed_refl = 0.0;
        // double spi_speed_refr = 0.0;

        controlmode_t mode;
        controlmode_t nextmode;
        controlmode_t prevmode;

        // ----- TIME -----
        int control_time;
};

extern SharedVariables shared;

#endif