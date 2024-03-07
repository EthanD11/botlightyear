// Import library
#include "Teensy.h"

// Note : Left = 1, Right = 2

// ----- GENERALS -----
// Current and reference x, y and theta
OutputInterface *outputs;
RobotPosition *robot_position;
PositionController *position_controller;
PathFollower *path_follower;
Regulator *speed_regulator;

controlmode_t mode = ModeIdle;

// ----- TIME -----
int control_time;

// --------------------------------------------------------------

void setup() {
  //Serial.begin(115200);
  // ----- SPI -----
  init_spi_interface();
  // ----- OUTPUTS -----
  outputs = init_outputs();
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

  if (spi_valid_transmission()) {
    spi_reset_transmission();
    switch (spi_get_query()) {
      case QueryPositionControl:
        spi_handle_position_control(robot_position, position_controller);
        mode = ModePositionControl;
        outputs->analog_write_a3pin = 256;
        break;

      case QuerySpeedControl:
        spi_handle_speed_control(speed_regulator);
        mode = ModeSpeedControl;
        outputs->analog_write_a3pin = 128;
        break;

      case QueryIdle:
        mode = ModeIdle;
        outputs->analog_write_a3pin = 64;
        break;
      
      default: // Idle
        mode = ModeIdle;
        outputs->analog_write_a3pin = 0;
        break;
    }
  }

  reset_encoders(robot_position);
  update_localization(robot_position);
 
  // Each 20ms TODO : Restimate REG_DELAY
  if(current_time - control_time > REG_DELAY){

    switch (mode) {
      case ModeIdle:
        control_time = current_time;
        break;

      case ModePositionControl:
        control_position(position_controller, robot_position);
        control_speed(speed_regulator,
          robot_position->speed_left, robot_position->speed_right,
          position_controller->speed_refl, position_controller->speed_refr);
        break;

      case ModeSpeedControl:
        break;

      default: // ModeIdle
        return;
    }

    write_outputs(outputs);

    // Update last control time
    control_time = current_time;
  }
}