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
	QueryDoPositionControl, // Position update, data received = [flag,x,y,t,xr,yr,tr]
	QueryDoSpeedControl,
	QueryDoSetDutyCycle,
	QueryDoPathFollowing,
	QueryDoConstantDutyCycle,
	QueryAskState,
	QuerySetPosition,
	QuerySetPositionControlGains,
	QuerySetPathFollowerGains
} query_t;

void init_spi_interface();
int spi_valid_transmission();
void spi_reset_transmission();
query_t spi_get_query();
void spi_set_state(uint32_t state_id);
void spi_handle_set_position(RobotPosition *robot_position);
void spi_handle_position_control(PositionController *position_controller); 
void spi_handle_path_following(PathFollower *path_follower);
void spi_handle_speed_control();
double spi_get_speed_refl();
double spi_get_speed_refr();
void spi_handle_constant_duty_cycle();
double spi_get_dc_refl();
double spi_get_dc_refr();
void spi_handle_set_position_control_gains(PositionController *position_control);
void spi_handle_set_path_follower_gains(PathFollower *path_follower);

// // Transform two bytes to a double
// // Works with the RPi if a single number is transmitted over 2 bytes
// // Returns a value between 0 and 1, scaling needs to be done after
// inline double decode_2byte_to_double(char byte1, char byte2) {
// 	char two_bytes[2];
// 	two_bytes[0] = byte1; // First byte
// 	two_bytes[1] = byte2; // Second byte
// 	uint16_t tmp_16 = *((uint16_t *) two_bytes); // Merge bytes
// 	return (((double) tmp_16)/UINT16_MAX); // Decode
// }
#endif