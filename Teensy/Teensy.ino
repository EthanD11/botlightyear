// Import library
#include <Encoder.h>
#include "SPISlave_T4.h"
#include "controller.h"

// Note : Left = 1, Right = 2

// ----- ENCODERS -----

// Create encoder objects with the pins A and B

Encoder enc_l(25, 26);
Encoder enc_r(30, 31);

// Level-shifter pin
const int LEVEL_SHIFTER = 2;

// Ticks
int old_tick_left = 0, old_tick_right = 0;
float speed_left = 0, speed_right = 0;
const float TICKS_TO_M = 1.3806e-6; // Multiply to get meters from tick count. pi*72e-3/20/8192

// ----- SPI -----

// Create spi object
SPISlave_T4 mySPI(0, SPI_8_BITS);


// ----- GENERALS -----

// Current and reference x, y and theta
float x = 0, y = 0, t = 0, xr = 0, yr = 0, tr = 0;
float fwd, rot;
float speed_refl, speed_refr;
// New values transferred over SPI channel, with associated flag for their arrival
uint32_t flag = 0;
uint8_t mode = 0;
uint32_t buf[7];
float *data;

// Time variables
int current_time;
int control_time;


typedef enum {
  Idle, // No input from RPi, default is to remain still
  New_position, // Received new t3 position data from RPi
  Following, // Pursuing current goal autonomously
  Remote_control // Follow speed command directly from RPi
} mode_t; // Control modes type

mode_t mode = Idle;


// --------------------------------------------------------------


void setup() {
  // Serial declaration
  Serial.begin(115200);
  while (!Serial);


  // ----- SPI -----
  mySPI.begin(MSBFIRST, SPI_MODE2);
  mySPI.onReceive(receiveEvent);


  // ----- MOTORS -----

  init_motors();

  // Default values
  control_time = millis();


  // ----- ENCODERS -----

  // Enable the level shifter
  pinMode(LEVEL_SHIFTER, OUTPUT);
  digitalWrite(LEVEL_SHIFTER, HIGH);

  // Reset encoders
  enc_l.write(0);
  enc_r.write(0);

}

void loop() {
  // Get time
  current_time = millis();

  // Each 20ms TODO : Restimate REG_DELAY
  if(current_time - control_time > REG_DELAY){

    // Update last control time
    control_time = current_time;

    // Updating values according to encoders
    int tick_left, tick_right;
    tick_left = enc_l.read(); tick_right = enc_r.read();

    // Temporarily forget delta t to avoid remultiplying when computing x, y and theta
    speed_left  = (tick_left  - old_tick_left )*TICKS_TO_M;
    speed_right = (tick_right - old_tick_right)*TICKS_TO_M;

    old_tick_left = tick_left; old_tick_right = tick_right;
    speed_left  /= (current_time - control_time);
    speed_right /= (current_time - control_time);
    

    switch (mode)
    {
    case Idle:

      speed_refl = 0; speed_refr = 0;
      break;

    case New_position:


      break;

    case Following:

      // Forward & rotation component
      fwd = (speed_left+speed_right)/2;
      rot = (speed_right-speed_left)/(2*WHEEL_L); // Divided by two, divided by half the distance between the two wheels = 176.17mm

      x += fwd*cos(t+rot/2);
      y += fwd*sin(t+rot/2);
      t += rot;
      t3_position_ctrl(x,y,t,xr,yr,tr, &speed_refl, &speed_refr);
      break;

    case Remote_control:

    default: // Idle
      speed_refl = 0; speed_refr = 0;
      break;
    }
      
    t1_speed_ctrl(speed_left, speed_right, speed_refl, speed_refr);

  }
}

void receiveEvent() {
  
  //When there is data to read
  while ( mySPI.available() ) {
    
    // Get data
    buf[flag++] = mySPI.popr();

    // Push number of data received in send buffer
    mySPI.pushr(flag);
    
  }

  if (flag == 7) {

    // Updating values according to SPI query
    data = (float*) buf;
    x  = data[0];
    y  = data[1];
    t  = data[2];
    xr = data[3];
    yr = data[4];
    tr = data[5];
    flag = 0;
  }
  
}
