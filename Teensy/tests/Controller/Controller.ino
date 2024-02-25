// Import library
#include <Encoder.h>
#include "controller.h"

#define T3

// Note : Left = 1, Right = 2

// ----- ENCODERS -----

// Create encoder objects with the pins A and B

Encoder enc_l(25, 26);
Encoder enc_r(30, 31);

// Level-shifter pin
const int LEVEL_SHIFTER = 2;

// Ticks
int old_tick_left = 0, old_tick_right = 0;
double speed_left = 0, speed_right = 0;
const double TICKS_TO_M = 1.3806e-6; // Multiply to get meters from tick count. pi*72e-3/20/8192


// ----- GENERALS -----

// Current and reference x, y and theta
#ifdef T3
double x = 0, y = 0, t = 0, xr = 1, yr = 0, tr = 0;
double fwd, rot;
#endif
double speed_refl=0.65, speed_refr=0.65;

// Time variables
int control_time;


// --------------------------------------------------------------


void setup() {
  // Serial declaration
  Serial.begin(115200);
  while (!Serial);

  // ----- MOTORS -----

  init_motors();

  // ----- ENCODERS -----

  // Enable the level shifter
  pinMode(LEVEL_SHIFTER, OUTPUT);
  digitalWrite(LEVEL_SHIFTER, HIGH);

  // Reset encoders
  enc_l.write(0);
  enc_r.write(0);

  // ----- GENERAL -----

  control_time = millis();

}

void loop() {
  // Get time
  int current_time = millis();

  // Each 20ms TODO : Restimate REG_DELAY
  if(current_time - control_time > REG_DELAY){

    if (current_time > 5000) { duty_cycle_update(0,0); return; }

    // Updating values according to encoders
    int tick_left, tick_right;
    tick_left = enc_l.read(); tick_right = enc_r.read();

    // Temporarily forget delta t to avoid remultiplying when computing x, y and theta
    speed_left  = (tick_left  - old_tick_left )*TICKS_TO_M;
    speed_right = (tick_right - old_tick_right)*TICKS_TO_M;

    old_tick_left = tick_left; old_tick_right = tick_right;

    #ifdef T3
    // Forward & rotation component
    fwd = (speed_left+speed_right)/2;
    rot = (speed_right-speed_left)/(2*WHEEL_L); // Divided by two, divided by half the distance between the two wheels = 176.17mm

    x += fwd*cos(t+rot/2);
    y += fwd*sin(t+rot/2);
    t += rot;
    t3_position_ctrl(x,y,t,xr,yr,tr, &speed_refl, &speed_refr);
    #endif

    speed_left  /= ((current_time - control_time)*1e-3);
    speed_right /= ((current_time - control_time)*1e-3);

    #ifdef VERBOSE
    printf("Speed left and right : %.4f, %.4f\n", speed_left, speed_right);
    #endif
    t1_speed_ctrl(speed_left, speed_right, speed_refl, speed_refr);

    // Update last control time
    control_time = current_time;

  }
}

