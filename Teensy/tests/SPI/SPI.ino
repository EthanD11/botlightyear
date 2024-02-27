/*
 * This example code shows how to use the spi communication bus
 */

const int a1 = 29;
const int A3 = 36;
int on1 = 0, on3 = 0;


// SPI library
#include "SPISlave_T4.h"
// Create spi object
// SPI_8_BITS -> Get each byte separately
SPISlave_T4 mySPI(0, SPI_8_BITS);

// Data to send to the SPI
uint32_t data = 13;

void setup() {
  // Serial declaration
  //Serial.begin(115200);
  //while (!Serial);

  // Configure SPI
  mySPI.begin(MSBFIRST, SPI_MODE0);
  mySPI.swapPins();
  mySPI.onReceive(receiveEvent);

  // Set pins to output
  pinMode(a1, OUTPUT);
  pinMode(A3, OUTPUT);

}

void loop() {
  // Nothing to do
}

void receiveEvent() {
  // Print message between [  ]
  Serial.print("[");
  //When there is data to read
  while ( mySPI.available() ) {
    // Get data
    data = mySPI.popr();

    if (data == 5) {
      on1 = 1-on1;
      if (on1) analogWrite(a1, 256);
      else analogWrite(a1, 0);
    }

    if (data == 1) {
      on3 = 1-on3;
      if (on3) analogWrite(a3, 256);
      else analogWrite(a3, 0);
    }

    // push it to send buffer
    mySPI.pushr(data);
    // Print data
    Serial.print(data, HEX);
  }
  Serial.println("]");
}
