// #include <Arduino.h>
#include "src/inout_interface/SPISlave_T4.h"
#include "src/inout_interface/output_interface.h"
#include "src/inout_interface/SPIInterface.h"
#include "src/localization/localization.h"
#include "src/path_follower/path_follower.h"
#include "src/position_control/position_control.h"
#include "src/regulator/regulator.h"
#include "utils.h"

// #define VERBOSE

typedef enum {
  ModeIdle, // No input from RPi, default is to remain still
  ModePositionControl,
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
// double spi_speed_refl = 0.0;
// double spi_speed_refr = 0.0;

controlmode_t mode = ModeIdle;
controlmode_t nextmode;

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
  robot_position = init_robot_position(0, 1.5, 0);
  // ----- SPEED REGULATOR -----
  speed_regulator = init_regulator();
  // ----- POSITION CONTROLLER -----
  position_controller = init_position_controller();
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
    printf("SPI received transmission\n");
    switch (spi_get_query()) {
      case QueryDoPathFollowing:
        printf("SPI QueryDoPathFollowing\n");
        spi_handle_path_following(path_follower);
        nextmode = ModePathFollowing;
        break;

      case QueryDoPositionControl:
        printf("SPI QueryDoPositionControl\n");
        spi_handle_position_control(position_controller);
        set_a3pin_duty_cycle(outputs, 0);
        nextmode = ModePositionControl;
        break;

      case QueryIdle:
        printf("SPI QueryIdle\n");
        set_a3pin_duty_cycle(outputs, 128);
        nextmode = ModeIdle;
        break;

      case QueryAskState:
        printf("SPI QueryAskState\n");
        nextmode = mode;
        break;

      case QuerySetPosition:
        printf("SPI QuerySetPosition\n");
        spi_handle_set_position(robot_position);
        nextmode = mode;
        break;
      
      default:
        printf("SPI QueryDefault\n");
        set_a3pin_duty_cycle(outputs, 255);
        nextmode = mode;
        break;
    }
    mode = nextmode;
  } 

  else if (current_time - control_time > REG_DELAY) {
    update_localization(robot_position);

    int ncheckpoints = 3;
    int path_following_goal_reached = 0;
    double x[5] = {0, 0.4, 0.5};
    double y[5] = {1.5, 1.5, 1.3};

    switch (mode) {
      case ModeIdle:
        #ifdef VERBOSE
        printf("\nMode idle\n");
        #endif
        set_motors_duty_cycle(outputs, 0, 0);
        break;

      case ModePositionControl:
        #ifdef VERBOSE
        printf("\nMode position control\n");
        #endif
        control_position(position_controller, robot_position);
        control_speed(speed_regulator, robot_position,
          get_speed_refl(position_controller), 
          get_speed_refr(position_controller));
        set_motors_duty_cycle(outputs, 
          get_duty_cycle_refl(speed_regulator), 
          get_duty_cycle_refr(speed_regulator));
        break;

      case ModePathFollowingInit:
        #ifdef VERBOSE
        printf("\nMode path following INIT\n");
        #endif
        init_path_following(path_follower, x, y, ncheckpoints, 0.0, M_PI);
        compute_entire_path(path_follower, 2e-3);
        delay(1);
        mode = ModePathFollowing;
        break;

      case ModePathFollowing:
        #ifdef VERBOSE
        printf("\nMode path following\n");
        printf("time = %d\n", current_time);
        #endif
        path_following_goal_reached = update_path_follower_ref_speed(path_follower, 
          robot_position, 25e-2, 10e-2);
        control_speed(speed_regulator, 
          robot_position,
          get_speed_refl(path_follower),
          get_speed_refr(path_follower));
        set_motors_duty_cycle(outputs,  
          get_duty_cycle_refl(speed_regulator), 
          get_duty_cycle_refr(speed_regulator));
        if (path_following_goal_reached) {
          set_ref(position_controller, 
            path_follower->last_x, 
            path_follower->last_y, 
            path_follower->last_theta);
        }
        break;

      default: // ModeIdle
        set_motors_duty_cycle(outputs, 0, 0);
        break;
    }
    
    #ifdef VERBOSE
    printf("dc left = %d\n", outputs->duty_cycle_l);
    printf("dc right = %d\n", outputs->duty_cycle_r);
    #endif

    // Next state logic
    switch (mode) {
      case ModeIdle:
        nextmode = ModeIdle;
        break;
      case ModePositionControl:
        nextmode = ModePositionControl;
        break;
      case ModePathFollowingInit:
        nextmode = ModePathFollowing;
        break;
      case ModePathFollowing:
        if (path_following_goal_reached) {
          nextmode = ModePositionControl;
          printf("Mode Position control is next\n");
        } else {
          nextmode = ModePathFollowing;
        }
        break;

      default:
        nextmode = ModeIdle;
        break;
    }
    write_outputs(outputs);
  }

  // Leave the current mode cleanly (free all malloc'd arrays and structs)
  if (nextmode != mode) {
    switch (mode) {
      case ModePathFollowing:
        close_path_following(path_follower);
        break;

      default:
        break;
    }
  }

  // Go to next mode and write outputs
  mode = nextmode;
  control_time = current_time;
}