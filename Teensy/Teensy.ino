// Import library
#include <Encoder.h>
#include "SPISlave_T4.h"
#include "controller.h"

//#define PARITY_CHECK

// Note : Left = 1, Right = 2

// ----- ENCODERS -----

// Create encoder objects with the pins A and B
// TODO : CHECK PINS
#ifdef ODOMETERS_ENC
Encoder enc_l(26, 25);
Encoder enc_r(31, 30);
#else
Encoder enc_l(25, 26);
Encoder enc_r(30, 31);
#endif

// Level-shifter pin
const int LEVEL_SHIFTER = 2;

// Ticks
int old_tick_left = 0, old_tick_right = 0;
double speed_left = 0, speed_right = 0;

#ifdef ODOMETERS_ENC
const double TICKS_TO_M = 1.7257e-5; // Multiply to get meters from tick count. pi*45e-3/8192
#else
const double TICKS_TO_M = 1.3806e-6; // Multiply to get meters from tick count. pi*72e-3/20/8192
#endif

// ----- SPI -----

typedef enum {
  QueryTestRead, // SPI test, answer with [1,2,3,4]
  QueryTestWrite, // SPI test, answer with data received
  QueryPositionControl, // Position update, data received = [flag,x,y,t,xr,yr,tr]
  QuerySpeedControl // Speed control, data received = [flag,left,right]
} query_t;

// Create spi object
SPISlave_T4 mySPI(0, SPI_8_BITS);

uint32_t i = 0, n = 7; // Number of bytes received, expected and actual
query_t query;
#ifdef PARITY_CHECK
uint8_t parity_bit;
#endif

uint32_t dataBuf[7];
//uint32_t testWriteBuf[4];




// ----- GENERALS -----

// Current and reference x, y and theta
double x = 0, y = 0, t = 0, xr = 0, yr = 0, tr = 0;
double fwd, rot;
double speed_refl, speed_refr;

// Time variables
int control_time;
//int request_time;
//const int SPI_TTL = 7*8/500;

typedef enum {
  ModeIdle, // No input from RPi, default is to remain still
  ModePositionControl,
  ModeSpeedControl
} controlmode_t; // Control modes type

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
  //request_time = -SPI_TTL;

}

void loop() {

  if (i == n) {
    i = 0;
    switch (dataBuf[0]) {
      case QueryPositionControl:
        enc_l.write(0); enc_r.write(0); 
        old_tick_left = 0; old_tick_right = 0;
        x  = ((double)(dataBuf[1]))*3/255;
        y  = ((double)(dataBuf[2]))*2/255;
        t  = ((double)(dataBuf[3]))*2*M_PI/255 - M_PI;
        xr = ((double)(dataBuf[4]))*3/255;
        yr = ((double)(dataBuf[5]))*2/255;
        tr = ((double)(dataBuf[6]))*2*M_PI/255 - M_PI;
        mode = ModePositionControl;
        break;

      case QuerySpeedControl:
        enc_l.write(0); enc_r.write(0); 
        old_tick_left = 0; old_tick_right = 0;
        speed_refl = ((double)dataBuf[1])*2/255;
        speed_refr = ((double)dataBuf[2])*2/255;
        mode = ModeSpeedControl;
        break;
      
      default:
        break;
    }
  }


  // Get time
  int current_time = millis();

  // Each 20ms TODO : Restimate REG_DELAY
  if(current_time - control_time > REG_DELAY){

    // Updating values according to encoders
    int tick_left, tick_right;
    tick_left = enc_l.read(); tick_right = enc_r.read();

    // Temporarily forget delta t to avoid remultiplying when computing x, y and theta
    speed_left  = (tick_left  - old_tick_left )*TICKS_TO_M;
    speed_right = (tick_right - old_tick_right)*TICKS_TO_M;

    old_tick_left = tick_left; old_tick_right = tick_right;

    #ifdef VERBOSE
    printf("Ticks : %d, %d\n", old_tick_left, old_tick_right);
    #endif

    switch (mode) {
    case ModeIdle:
      duty_cycle_update(0,0);
      return;

    case ModePositionControl:

      // Forward & rotation component
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
      duty_cycle_update(0,0);
      return;
    }

    speed_left  /= ((current_time - control_time)*1e-3);
    speed_right /= ((current_time - control_time)*1e-3);
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

        default:
          n = 7;
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
