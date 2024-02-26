/*
 * This example code shows how to use the spi communication bus
 */

const int A1 = 29;
const int A3 = 36;
int on = 0;


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
  pinMode(A1, OUTPUT);
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
    // push it to send buffer
    if (data == 7) analog
    mySPI.pushr(data);
    // Print data
    Serial.print(data, HEX);
  }
  Serial.println("]");
}
