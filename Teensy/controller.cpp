// Includes and inlines
#include <Arduino.h>
#include "controller.h"
#include <cmath>
inline int ABS(int x) {
  return ((x >= 0) ? x : -x);
}
inline double ABS(double x) {
  return ((x >= 0) ? x : -x);
}
inline int SAT(int x, int limit) {
  return ((x > limit) ? limit : ((x < -limit) ? -limit : x));
}  // Saturation function for integers
inline double SAT(double x, double limit) {
  return ((x > limit) ? limit : ((x < -limit) ? -limit : x));
}  // Saturation function for doubles


#define MOTOR_DUTY_RANGE 250  // Duty cycle range
#define BUF_STEP 250           // Max step to avoid brutal speed changes

// Uncomment to enable Anti-Dead Zone (ADZ)
// Do not use this, it is unnecessary and unstable (and needs to be changed anyway)
//#define ADZ_ENABLE


#ifdef ADZ_ENABLE
int adz = 5;
#endif


// Pins
// IN1
const uint8_t C_L = 14, C_R = 3;
// IN2
const uint8_t D_L = 15, D_R = 4;
// Enable
const uint8_t PWM_L = 22, PWM_R = 23;
// Current sensors
//const uint8_t CURRENT_L = 41, CURRENT_R = 40;

// Parameters definiton
// TODO : Recompute for Teensy
// -- T1 --
const double t1_kp = 7.795856e-01;                     // Proportional coefficient for speed (V/(rad_mot/s))
const double t1_ki = 8.398411e-01 * REG_DELAY * 1e-3;  // Integral coefficient for speed (V/rad_motor) * Delta t for the integral
const double t1_aw = 30;                              // Saturation level of the integral (anti-windup)
// -- T3 --
// Note : local asymptocical stability if ka > kp > 0, kb < 0
const double t3_kp = 0.7;          // Proportional coefficient for distance error
const double t3_ka = 3.5;          // Proportional coefficient for direction error
const double t3_kb = -0.7;         // Proportional coefficient for orientation error
const double t3_pos_tol = 1e-1;  // Acceptable static error on position (m)
const double t3_dft_tol = 2e-1;  // Acceptable drift from reference position when reorienting (m)
const double t3_ang_tol = 8.73e-2; // Acceptable static error on orientation (rad, eq to 5 degrees)
int flag_reached = 0;

double isl = 0, isr = 0;        // Integrals of error on speed, left and right
int dc_curl = 0, dc_curr = 0;  // Current duty cycle values

void t1_speed_ctrl(double speed_l, double speed_r, double ref_l, double ref_r) {

  speed_l = SAT(speed_l, REF_SPEED_LIMIT);
  speed_r = SAT(speed_r, REF_SPEED_LIMIT);

  // PI controller
  double esl, esr;        // Errors on speed, left and right
  double vl, vr;          // Voltage output commands, left and right
  int dc_refl, dc_refr;  // Duty cycles

#ifdef ADZ_ENABLE
  int azl, azr;  // Anti dead zone term
#endif

  // Compute error
  esl = ref_l - speed_l;
  esr = ref_r - speed_r;

  // Compute integral
  isl = SAT(isl + esl, t1_aw);
  isr = SAT(isr + esr, t1_aw);

#ifdef VERBOSE
  printf("Left Integral  : %.4f\n", isl);
  printf("Right Integral : %.4f\n", isr);
#endif

  // Compute voltages
  vl = t1_kp * esl + t1_ki * isl;
  vr = t1_kp * esr + t1_ki * isr;

// update duty cycle, assuming duty cycle changes average voltage linearly
#ifdef ADZ_ENABLE
  // add an anti-deadzone term to compensate the deadzone around 0
  // Do not use this, it is unnecessary and dangerous
  int adz_l = (vl < 0) ? -adz : adz, adz_r = (vr < 0) ? -adz : adz;
  dc_refl = SAT(((int)(vl * MOTOR_DUTY_RANGE)) + adz_l, MOTOR_DUTY_RANGE);
  dc_refr = SAT(((int)(vr * MOTOR_DUTY_RANGE)) + adz_r, MOTOR_DUTY_RANGE);
#else
  dc_refl = SAT((int)(vl * MOTOR_DUTY_RANGE), MOTOR_DUTY_RANGE);
  dc_refr = SAT((int)(vr * MOTOR_DUTY_RANGE), MOTOR_DUTY_RANGE);
#endif
#ifdef VERBOSE
  printf("Voltages : %.4f, %.4f\n", vl*24, vr*24);
  printf("Updating duty cycles to : %d, %d\n\n", dc_refl, dc_refr);
#endif

  duty_cycle_update(dc_refl, dc_refr);
}


void t3_position_ctrl(double x, double y, double t, double xr, double yr, double tr, double *ref_l, double *ref_r) {

  // Inspired by, but not identical to :
  // https://moodle.uclouvain.be/pluginfile.php/41211/mod_resource/content/1/Mobile_robots_control_2015.pdf?forcedownload=0
  // Slides 22-27

  #ifdef VERBOSE
  printf("Current coordinates : %.4f, %.4f, %.4f\n", x, y, t);
  printf("Reference coordinates : %.4f, %.4f, %.4f\n", xr, yr, tr);
  #endif

  double dx, dy; // Errors in cartesian coordinates
  double p, phi, a, b; // Errors on position and orientation in "polar" coordinates
  double v_ref, rot_ref; // Reference velocity and rotation
  //double temp;

  // Comute the errors in standard coordinates
  dx = xr - x;
  dy = yr - y;

  // Compute the errors in "polar" coordinates
  p = hypot(dx, dy);
  if (p < t3_pos_tol) flag_reached = 1;
  else if (p > t3_dft_tol) flag_reached = 0;

  if (flag_reached) {
    // If the distance to the goal is acceptably small, assume goal is reached (p = a = 0)
    // Only the error on orientation remains
    a = 0;
    p = 0;
    b = t - tr;
  } else {
    // Compute shortest path around the circle
    // Derived by @Kakoo :)
    phi = atan2(dy, dx);
    a = phi - t;
    b = tr - phi;  // b = tr - t - a
    if (ABS(a) > PI) a -= ((a > 0) ? 1 : -1) * 2 * M_PI;
  }

  if (ABS(b) > PI) b -= ((b > 0) ? 1 : -1) * 2 * M_PI;
  if (ABS(b) < t3_ang_tol) b = 0;

#ifdef VERBOSE
  printf("Errors in polar coordinates : %.3f, %.3f, %.3f\n", p, a, b);
#endif

  // Compute reference velocity and rotation
  // Temp allows for a smoother transition, will be renamed if kept
  //temp = 0.557 * ((a + 1.22)/(0.14 + ABS(a + (double) 1.22)) + (1.22 - a)/(0.14 + ABS(a - (double) 1.22)));
  v_ref = t3_kp * p;
  rot_ref = t3_ka * a + t3_kb * b;

  // Translate into left and right wheel reference speed
  *ref_l = v_ref - WHEEL_L * rot_ref;
  *ref_r = v_ref + WHEEL_L * rot_ref;
}

void init_motors() {

  // Output pins
  pinMode(C_L, OUTPUT);
  pinMode(D_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(C_R, OUTPUT);
  pinMode(D_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  // Input pins
  //pinMode(CURRENT_L, INPUT);
  //pinMode(CURRENT_R, INPUT);

  digitalWrite(C_L, LOW);
  digitalWrite(D_L, HIGH);
  digitalWrite(C_R, HIGH);
  digitalWrite(D_R, LOW);
  
  analogWriteFrequency(PWM_L, 20e3);
  analogWriteFrequency(PWM_R, 20e3);
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);
}

void duty_cycle_update(int left, int right) {

  // Left buffered control
  dc_curl += SAT(left - dc_curl, BUF_STEP);
  analogWrite(PWM_L, ABS(dc_curl));

  // Right buffered control
  dc_curr += SAT(right - dc_curr, BUF_STEP);
  analogWrite(PWM_R, ABS(dc_curr));

  // Left Direction (forward vs backward)
  if (dc_curl < 0) {
    digitalWrite(C_L, HIGH);
    digitalWrite(D_L, LOW);
  } else {
    digitalWrite(C_L, LOW);
    digitalWrite(D_L, HIGH);
  }

  // Right Direction (forward vs backward)
  if (dc_curr < 0) {
    digitalWrite(C_R, LOW);
    digitalWrite(D_R, HIGH);
  } else {
    digitalWrite(C_R, HIGH);
    digitalWrite(D_R, LOW);
  }
}