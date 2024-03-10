#ifndef _SPIINTERFACE_H_
#define _SPIINTERFACE_H_

#include "../../utils.h"
#include "SPISlave_T4.h"
#include "../position_control/position_control.h"
#include "../regulator/regulator.h"
#include "../localization/localization.h"
#include "../path_follower/path_follower.h"

#ifdef PARITY_CHECK
uint8_t parity_bit;
#endif


typedef enum {
	QueryIdle, // Idle, reset motor voltages to 0V
	QueryTestRead, // SPI test, answer with [1,2,3,4]
	QueryTestWrite, // SPI test, answer with data received
	QueryDoPositionControl, // Position update, data received = [flag,x,y,t,xr,yr,tr]
	QueryDoPathFollowing,
	QueryAskGoalReached
} query_t;

void init_spi_interface();
int spi_valid_transmission();
void spi_reset_transmission();
query_t spi_get_query();
void spi_handle_position_control(RobotPosition *robot_position, PositionController *position_controller); 
void spi_handle_path_following(PathFollower *path_follower);

#endif