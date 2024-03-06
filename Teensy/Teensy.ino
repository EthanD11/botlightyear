// Import library
#include <Encoder.h>
#include "SPISlave_T4.h"
#include "SPIInterface.h"
// #include "controller.h"
#include "localization.h"
#include "path_follower.h"
#include "position_control.h"
#include "regulator.h"
#include "output_interface.h"

// Note : Left = 1, Right = 2

// ----- GENERALS -----
// Current and reference x, y and theta
OutputInterface *outputs;
SPIInterface *spi_interface;

RobotPosition *robot_position;
PositionController *position_controller;
PathFollower *path_follower;
Regulator *speed_regulator;

typedef enum {
  ModeIdle, // No input from RPi, default is to remain still
  ModePositionControl,
  ModeSpeedControl
} controlmode_t; // Control modes type
controlmode_t mode = ModeIdle;

// ----- TIME -----
int control_time;

// --------------------------------------------------------------

void setup() {
  //Serial.begin(115200);

  // ----- OUTPUTS -----
  outputs = init_outputs();

  // ----- SPI -----
  spi_interface = init_spi_interface();

  // ----- POSITION -----
  robot_position = init_robot_position(0, 0, 0);

  // ----- POSITION CONTROLLER -----
  position_controller = init_position_controller();

  // ----- SPEED REGULATOR -----
  speed_regulator = init_regulator();

  // ----- PATH FOLLOWER -----
  path_follower = init_path_follower();

  // ----- GENERAL -----
  control_time = millis();

}

void loop() {
    
  // Get time
  int current_time = millis();
  robot_position->dt = 1e-3*(current_time - control_time);

  if (i == n) {
    i = 0;
    switch (spi_interface->query) {
      case QueryPositionControl:
        reset_encoders(robot_position);
        set_position(robot_position, &spi_interface->data_buffer[0]);
        set_position_reference(position_controller, &spi_interface->data_buffer[4]);
        mode = ModePositionControl;
        outputs->analog_write_a3pin = 256;
        break;

      case QuerySpeedControl:
        reset_encoders(robot_position);
        speed_regulator->speed_refl = ((double)dataBuf[1])*2/255;
        speed_regulator->speed_refr = ((double)dataBuf[2])*2/255;
        mode = ModeSpeedControl;
        outputs->analog_write_a3pin = 128;
        break;

      case QueryIdle:
        mode = ModeIdle;
        outputs->analog_write_a3pin = 0;
        break;
      
      default: // Idle
        mode = ModeIdle;
        outputs->analog_write_a3pin = 0;
        break;
    }
  }

  update_localization(robot_position);
 
  // Each 20ms TODO : Restimate REG_DELAY
  if(current_time - control_time > REG_DELAY){

    switch (mode) {
      case ModeIdle:
        control_time = current_time;
        duty_cycle_update(motors, 0, 0);
        return;

      case ModePositionControl:
        control_position(position_controller, robot_position);
        control_speed(speed_regulator,
          robot_position->speed_left, robot_position->speed_right,
          position_controller->speed_refl, position_controller->speed_refr);
        break;

      case ModeSpeedControl:
        break;

      default: // ModeIdle
        duty_cycle_update(motors,0,0);
        return;
    }

    duty_cycle_update(motors, 
          speed_regulator->duty_cycle_refl,
          speed_regulator->duty_cycle_refr);

    // Update last control time
    control_time = current_time;
  }
}