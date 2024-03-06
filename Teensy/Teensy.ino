// Import library
#include <Encoder.h>
#include "SPISlave_T4.h"
#include "controller.h"
#include "localization.h"
#include "path_follower.h"

//#define PARITY_CHECK

// Note : Left = 1, Right = 2

// ----- ENCODERS -----

// Create encoder objects with the pins A and B
// Level-shifter pin
const int LEVEL_SHIFTER = 2;


// ----- Path Follower -----
PathFollower *path_follower;

// ----- SPI -----

// Test pin
const uint8_t a3 = 36;

typedef enum {
  QueryIdle, // Idle, reset motor voltages to 0V
  QueryTestRead, // SPI test, answer with [1,2,3,4]
  QueryTestWrite, // SPI test, answer with data received
  QueryPositionControl, // Position update, data received = [flag,x,y,t,xr,yr,tr]
  QuerySpeedControl // Speed control, data received = [flag,left,right]
} query_t;

// Create spi object
SPISlave_T4 mySPI(0, SPI_8_BITS);

uint32_t i = 0, n = 1; // Number of bytes received, expected and actual
query_t query;
#ifdef PARITY_CHECK
uint8_t parity_bit;
#endif

uint32_t dataBuf[7];
//uint32_t testWriteBuf[4];

// ----- GENERALS -----
// Current and reference x, y and theta
RobotPosition *robot_position;
double xr = 0, yr = 0, tr = 0;
double fwd, rot;
double speed_refl, speed_refr;

// Time variables
int control_time;
//int request_time;
//const int SPI_TTL = 7*8/500;

controlmode_t mode = ModeIdle;


// --------------------------------------------------------------


void setup() {
  // Serial declaration
  //Serial.begin(115200);
  //while (!Serial);

  // ----- SPI -----
  mySPI.begin(MSBFIRST, SPI_MODE0);
  mySPI.swapPins();
  mySPI.onReceive(receiveEvent);
  pinMode(a3, OUTPUT);
  analogWriteFrequency(a3,20e3);
  analogWrite(a3,0);

  // ----- MOTORS -----

  init_motors();

  // ----- ENCODERS -----

  // Enable the level shifter
  pinMode(LEVEL_SHIFTER, OUTPUT);
  digitalWrite(LEVEL_SHIFTER, HIGH);

  robot_position = init_robot_position(0, 0, 0);

  // ----- PATH FOLLOWER -----
  path_follower = init_path_follower();

  // ----- GENERAL -----
  control_time = millis();
  //request_time = -SPI_TTL;

}

void loop() {
    
  // Get time
  int current_time = millis();
  robot_position->dt = 1e-3*(current_time - control_time);

  if (i == n) {
    i = 0;
    switch (dataBuf[0]) {
      case QueryPositionControl:
        reset_encoders(robot_position);
        set_position(robot_position, dataBuf);
        xr = ((double)(dataBuf[4]))*3/255;
        yr = ((double)(dataBuf[5]))*2/255;
        tr = ((double)(dataBuf[6]))*2*M_PI/255 - M_PI;
        mode = ModePositionControl;
        analogWrite(a3, 256);
        break;

      case QuerySpeedControl:
        reset_encoders(robot_position);
        speed_refl = ((double)dataBuf[1])*2/255;
        speed_refr = ((double)dataBuf[2])*2/255;
        mode = ModeSpeedControl;
        analogWrite(a3, 128);
        break;

      case QueryIdle:
        mode = ModeIdle;
        analogWrite(a3, 0);
        break;
      
      default: // Idle
        mode = ModeIdle;
        analogWrite(a3, 0);
        break;
    }
  }

  update_localization(robot_position);
 
  // Each 20ms TODO : Restimate REG_DELAY
  if(current_time - control_time > REG_DELAY){

    switch (mode) {
      case ModeIdle:
        control_time = current_time;
        duty_cycle_update(0,0);
        return;

      case ModePositionControl:

        // Update position estimate from encoder data
        fwd = (speed_left+speed_right)/2;
        rot = (speed_right-speed_left)/(2*WHEEL_L); // Divided by two, divided by half the distance between the two wheels = 176.17mm

        x += fwd*cos(t+rot/2);
        y += fwd*sin(t+rot/2);
        t += rot;
        t3_position_ctrl(x,y,t,xr,yr,tr, &speed_refl, &speed_refr);
        break;

      case ModeSpeedControl:
        break;

      default: // ModeIdle
        control_time = current_time;
        duty_cycle_update(0,0);
        return;
    }

    speed_left  /= ((current_time - control_time)*1e-3);
    speed_right /= ((current_time - control_time)*1e-3);
    if ((std::abs(speed_refl) < SPD_TOL) && (std::abs(speed_refr) < SPD_TOL) && (std::abs(speed_left) < SPD_TOL) && (std::abs(speed_right) < SPD_TOL)) mode = ModeIdle;
    #ifdef VERBOSE
    printf("Speed : %.4f\t%.4f\n", speed_left, speed_right);
    printf("Speed reference : %.4f\t%.4f\n", speed_refl, speed_refr);
    #endif

    t1_speed_ctrl(speed_left, speed_right, speed_refl, speed_refr);

    // Update last control time
    control_time = current_time;

  }
}

void receiveEvent() {
  
  //When there is data to read
  while ( mySPI.available() ) {

    uint32_t data = mySPI.popr();

    if (i == 0) {
      switch (data) {
        case QueryPositionControl :
          n = 7;
          break;

        case QuerySpeedControl:
          n = 3;
          break;

        case QueryIdle:
          n = 1;
          break;

        default: // Idle
          n = 1;
          break;
      }
    }
    // Get data
    dataBuf[i++] = data;

    #ifdef VERBOSE
    printf("NEW DATA : %d\n", (int) data);
    #endif
  }
}

#ifdef PARITY_CHECK
inline uint8_t parity_check() {
  uint8_t b = 0;
  for (int j = 0, j < n, i++) {
    b ^= __builtin_parity(dataBuf[j]);
  }
  return b;
} 
#endif
