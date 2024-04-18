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
// #define SWITCH_VERBOSE
// #define SPI_VERBOSE

typedef enum {
  ModeIdle, // No input from RPi, default is to remain still
  ModePositionControl,
  ModePathFollowingInit,
  ModePathFollowing,
  ModeSpeedControl,
  ModeConstantDC,
  ModePositionControlOver
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
controlmode_t nextmode = mode;

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
  // ----- SPEED REGULATOR -----
  speed_regulator = init_regulator();
  // ----- POSITION CONTROLLER -----
  position_controller = init_position_controller();
  set_ref(position_controller, 0.5, 0, 0);
  // ----- PATH FOLLOWER -----
  path_follower = init_path_follower();
  // ----- GENERAL -----
  control_time = micros();

  if (mode == ModePathFollowingInit) {
    printf("In setup: ModePathFollowingInit\n");
  }

  set_apins(outputs, (int8_t) mode);
}

void loop() {
  int current_time = micros();
  int switch_mode = FALSE;
  robot_position->dt = 1e-6*((double)(current_time - control_time));
  
  if (spi_valid_transmission()) {
    spi_reset_transmission(); 
    #ifdef SPI_VERBOSE
    printf("SPI received transmission\n");
    #endif
    switch (spi_get_query()) {
      case QueryDoPathFollowing:
        #ifdef SPI_VERBOSE
        printf("SPI QueryDoPathFollowing\n");
        #endif
        spi_handle_path_following(path_follower);
        nextmode = ModePathFollowing;
        break;

      case QueryDoPositionControl:
        #ifdef SPI_VERBOSE
        printf("SPI QueryDoPositionControl\n");
        #endif
        spi_handle_position_control(position_controller);
        nextmode = ModePositionControl;
        break;

      case QueryDoSpeedControl:
        #ifdef SPI_VERBOSE
        printf("SPI SpeedControl\n");
        #endif
        spi_handle_speed_control();
        nextmode = ModeSpeedControl;
        break;

      case QueryIdle:
        #ifdef SPI_VERBOSE
        printf("SPI QueryIdle\n");
        #endif
        nextmode = ModeIdle;
        break;

      case QueryAskState:
        #ifdef SPI_VERBOSE
        printf("SPI QueryAskState\n");
        #endif
        nextmode = mode;
        break;

      case QuerySetPosition:
        #ifdef SPI_VERBOSE
        printf("SPI QuerySetPosition\n");
        #endif
        spi_handle_set_position(robot_position);
        nextmode = mode;
        break;

      case QueryDoConstantDutyCycle:
        #ifdef SPI_VERBOSE
        printf("SPI Constant DC\n");
        #endif
        spi_handle_constant_duty_cycle();
        nextmode = ModeConstantDC;
        break;

      case QuerySetPositionControlGains:
        #ifdef SPI_VERBOSE
        printf("SPI Set Position Control Gains\n");
        #endif
        spi_handle_set_position_control_gains(position_controller);
        nextmode = mode;
        break;

      case QuerySetPathFollowerGains:
        #ifdef SPI_VERBOSE
        printf("SPI Set Path Follower Gains\n");
        #endif
        spi_handle_set_path_follower_gains(path_follower);
        nextmode = mode;
        break;
      
      default:
        #ifdef SPI_VERBOSE
        printf("SPI QueryDefault\n");
        #endif
        nextmode = mode;
        break;
    }
    switch_mode = (mode != nextmode);
    mode = nextmode;
  } else if (current_time - control_time >= REG_DELAY) {

    update_localization(robot_position);
    int ncheckpoints = 5;
    int path_following_goal_reached = 0;
    double x[5] = {0.0,0.4,0.8,0.4,0.0};
    double y[5] = {0.0,0.2,0.0,-0.2,0.0};
    // printf("mode = %d\n", (int) mode);
    spi_set_state((uint32_t) mode);

    // Output logic
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

       case ModePositionControlOver:
        #ifdef VERBOSE
        printf("\nMode position control over\n");
        #endif
        control_speed(speed_regulator, robot_position, 0, 0);
        set_motors_duty_cycle(outputs,  
          get_duty_cycle_refl(speed_regulator), 
          get_duty_cycle_refr(speed_regulator));
        break;

      case ModeSpeedControl:
        #ifdef VERBOSE
        printf("\nModeSpeedControl\n");
        #endif
        control_speed(speed_regulator, robot_position,
          spi_get_speed_refl(), spi_get_speed_refr());
        set_motors_duty_cycle(outputs,
          get_duty_cycle_refl(speed_regulator), 
          get_duty_cycle_refr(speed_regulator));
        break;

      case ModeConstantDC:
        #ifdef VERBOSE
        printf("\nMode Constant DC\n");
        #endif
        set_motors_duty_cycle(outputs,
          spi_get_dc_refl(), spi_get_dc_refr());
        break;

      case ModePathFollowingInit:
        #ifdef VERBOSE
        printf("\nMode path following INIT\n");
        #endif
        init_path_following(path_follower, x, y, ncheckpoints, 0.0, M_PI, 0.2, 0.1);
        break;

      case ModePathFollowing:
        #ifdef VERBOSE
        printf("\nMode path following\n");
        #endif
        path_following_goal_reached = update_path_follower_ref_speed(path_follower, 
          robot_position);
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
        #ifdef VERBOSE
        printf("Default in output logic\n");
        #endif
        set_motors_duty_cycle(outputs, 0, 0);
        break;
    }
    
    // Print logic ;-)
    #ifdef VERBOSE
    // printf("dt = %.5e\n", ((double) 1e-6*current_time));
    printf("xpos = %.5e\n", robot_position->x);
    printf("ypos = %.5e\n", robot_position->y);
    printf("thetapos = %.5e\n", robot_position->theta);
    // printf("kif = %.5e\n", path_follower->kif);
    // printf("vfwd = %.5e\n", robot_position->vfwd);
    // printf("omega = %.5e\n", robot_position->omega);
    printf("speed_left = %.5e\n", robot_position->speed_left);
    printf("speed_right = %.5e\n", robot_position->speed_right);
    printf("dc left = %d\n", outputs->duty_cycle_l);
    printf("dc right = %d\n", outputs->duty_cycle_r);
    // printf("el_filtered = %.5e\n", speed_regulator->el_filtered);
    // printf("er_filtered = %.5e\n", speed_regulator->er_filtered);
    // printf("isl = %.5e\n", speed_regulator->isl);
    // printf("isr = %.5e\n", speed_regulator->isr);

    switch (mode) {
      case ModePathFollowing:
        printf("xref = %.5e\n", path_follower->xref);
        printf("yref = %.5e\n", path_follower->yref);
        printf("qref = %.5e\n", path_follower->qref);
        printf("et = %.5e\n", path_follower->et);
        printf("en = %.5e\n", path_follower->en);
        printf("z = %.5e\n", path_follower->z);
        break;

      default:
        printf("et = NaN\n");
        printf("en = NaN\n");
        printf("z = NaN\n");
        break;
    }

    switch (mode) {
      case ModePathFollowing:
        printf("vref = %.6e\n", path_follower->vref);
        printf("omega_ref = %.6e\n", path_follower->omega_ref);
        break;

      case ModePositionControl:
        printf("vref = %.6e\n", position_controller->vref);
        printf("omega_ref = %.6e\n", position_controller->omega_ref);
        break;

      case ModePositionControlOver:
        printf("vref = %.6e\n", 0.0);
        printf("omega_ref = %.6e\n", 0.0);
        break;

      default:
        printf("vref = NaN\n");
        printf("omega_ref = NaN\n");
        break;
    }
    
    switch (mode) {
      case ModeSpeedControl:
        printf("ref_speedl = %.6e\n", spi_get_speed_refl());
        printf("ref_speedr = %.6e\n", spi_get_speed_refr());
        break;

      case ModePositionControl:
        printf("ref_speedl = %.6e\n", get_speed_refl(position_controller));
        printf("ref_speedr = %.6e\n", get_speed_refr(position_controller));
        break;

      case ModePathFollowing:
        printf("ref_speedl = %.6e\n", get_speed_refl(path_follower));
        printf("ref_speedr = %.6e\n", get_speed_refr(path_follower));
        break;

      default:
        printf("ref_speedl = NaN\n");
        printf("ref_speedr = NaN\n");
        break;
    }
    #endif

    // Next state logic
    switch (mode) {
      case ModeIdle:
        nextmode = ModeIdle;
        break;
      case ModePositionControl:
        if (position_controller->flag_angular_position_reached && position_controller->flag_position_reached) {
          nextmode = ModePositionControlOver;
        } else {
          nextmode = ModePositionControl;
        }
        break;
      case ModePositionControlOver:
        nextmode = ModePositionControlOver;
        break;
      case ModePathFollowingInit:
        nextmode = ModePathFollowing;
        break;
      case ModePathFollowing:
        if (path_following_goal_reached) {
          nextmode = ModePositionControl;
        } else {
          nextmode = ModePathFollowing;
        }
        break;
      case ModeSpeedControl:
        nextmode = ModeSpeedControl;
        break;
      case ModeConstantDC:
        nextmode = ModeConstantDC;
        break;

      default:
        nextmode = ModeIdle;
        break;
    }
    write_outputs(outputs);
    control_time = current_time;

    switch_mode = (mode != nextmode);
    mode = nextmode;
  }

  // Leave the current mode cleanly (free all malloc'd arrays and structs)
  if (switch_mode) {
    reset_regulator(speed_regulator);
    reset_encoders(robot_position);
    
    set_apins(outputs, (int8_t) mode);
    switch (mode) {
      case ModePathFollowing:
        close_path_following(path_follower);
        break;

      case ModePositionControlOver:
        position_controller->flag_angular_position_reached = FALSE;
        position_controller->flag_position_reached = FALSE;
        break;

      default:
        break;
    }

    #ifdef SWITCH_VERBOSE
    switch (mode) {
      case ModeIdle:
        printf("Switch to mode idle\n");
        break;
      case ModePositionControl:
        printf("Switch to mode position control\n");
        printf("xpos = %.5e\n", robot_position->x);
        printf("ypos = %.5e\n", robot_position->y);
        printf("thetapos = %.5e\n", robot_position->theta);
        break;
      case ModePositionControlOver:
        printf("Switch to mode position control over\n");
        break;
      case ModePathFollowingInit:
        printf("Switch to mode path following init\n");
        break;
      case ModePathFollowing:
        printf("Switch to mode path following\n");
        printf("xpos = %.5e\n", robot_position->x);
        printf("ypos = %.5e\n", robot_position->y);
        printf("thetapos = %.5e\n", robot_position->theta);
        break;
      case ModeSpeedControl:
        printf("Switch to mode speed control\n");
        break;
      case ModeConstantDC:
        printf("Switch to mode constant dc\n");
        break;

      default:
        printf("Switch to mode unknown\n");
        break;
    } 
    #endif
  }
  
}