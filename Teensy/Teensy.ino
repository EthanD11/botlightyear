#include "Teensy.h"

typedef enum {
  ModeIdle, // No input from RPi, default is to remain still
  ModePositionControl,
  ModeSpeedControl,
  ModePathFollowingInit,
  ModePathFollowing
} controlmode_t; // Control modes type
// Note : Left = 1, Right = 2

// ----- GENERALS -----
// Current and reference x, y and theta
OutputInterface *outputs;
RobotPosition *robot_position;
PositionController *position_controller;
PathFollower *path_follower;
Regulator *speed_regulator;
double spi_speed_refl = 0.0;
double spi_speed_refr = 0.0;

controlmode_t mode = ModePathFollowingInit;

// ----- TIME -----
int control_time;

// --------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.print("Setting up teensy\n");
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
  robot_position->dt = 1e-3*((double)(current_time - control_time));

  if (spi_valid_transmission()) {
    spi_reset_transmission();   
    switch (spi_get_query()) {
      case QueryPositionControl:
        spi_handle_position_control(robot_position, position_controller);
        mode = ModePositionControl;
        outputs->analog_write_a3pin = 0;
        break;

      case QuerySpeedControl:
        spi_handle_speed_control(&spi_speed_refl, &spi_speed_refr);
        mode = ModeSpeedControl;
        outputs->analog_write_a3pin = 0;
        break;

      case QueryIdle:
        mode = ModeIdle;
        outputs->analog_write_a3pin = 255;
        Serial.print("Mode Idle");
        break;
      
      default: // Idle
        mode = mode;
        outputs->analog_write_a3pin = 255;
        break;
    }
  }

  
  // printf("speed left: %.3e\n", robot_position->speed_left);
  // printf("speed right: %.3e\n", robot_position->speed_right);
  int ncheckpoints = 5;
  double x[5] = {0, 0.4, .8, 0.4, 0.0};
  double y[5] = {0, 0.20, 0.0, -0.20, 0.0};
  // Each 20ms TODO : Restimate REG_DELAY
  if(current_time - control_time > REG_DELAY){
    update_localization(robot_position);
    
    
    switch (mode) {
      case ModeIdle:
        // printf("\nMode idle\n");
        control_time = current_time;
        outputs->analog_write_a3pin = 128;
        break;

      case ModePositionControl:
        printf("\nMode position control\n");
        control_position(position_controller, robot_position);
        control_speed(
          speed_regulator, outputs, robot_position,
          position_controller->speed_refl, 
          position_controller->speed_refr);
        break;

      case ModeSpeedControl:
        printf("Mode speed control\n");
        control_speed(speed_regulator, outputs, robot_position, 0.1, 0.1);
        break;

      case ModePathFollowingInit:
        printf("\nMode path following INIT\n");
        init_path_following(path_follower, x, y, ncheckpoints, 0.0);
        compute_entire_path(path_follower, 2e-3);
        // for (int i = 0; i < path_follower->m; i++) {
        //   printf("%d,%.6e,%.6e\n",i, path_follower->path[2*i], path_follower->path[2*i+1]);
        // }
        delay(1);
        mode = ModePathFollowing;
        break;

      case ModePathFollowing:
        printf("\nMode path following\n");
        printf("time = %d\n", current_time);
        int retval;
        retval = update_path_follower_ref_speed(path_follower, robot_position, 40e-2, 5e-2);
        control_speed(speed_regulator, outputs, robot_position,
          path_follower->speed_refl,
          path_follower->speed_refr);
        if (retval == 1) {
          position_controller->xref = path_follower->checkpoints_x[path_follower->n-1];
          position_controller->yref = path_follower->checkpoints_y[path_follower->n-1];
          position_controller->theta_ref = M_PI;
          mode = ModePositionControl;
        }
        break;

      default: // ModeIdle
        outputs->duty_cycle_refl = 0;
        outputs->duty_cycle_refr = 0;
        break;
    }

    // outputs->duty_cycle_refl = 0;
    // outputs->duty_cycle_refr = 0;
    // printf("outputs->duty_cycle_left =%d\n", outputs->duty_cycle_l);
    // printf("outputs->duty_cycle_right =%d\n", outputs->duty_cycle_r);

    write_outputs(outputs);

    // Update last control time
    control_time = current_time;
  }
}