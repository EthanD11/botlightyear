/*
 * This example code shows how to use the spi communication bus
 */

// SPI library
#include "SPISlave_T4.h"
// Create spi object
// SPI_8_BITS -> Get each byte separately
SPISlave_T4 mySPI(0, SPI_8_BITS);

// Data to send to the SPI
uint32_t data = 13;

void setup() {
  // Serial declaration
  Serial.begin(115200);
  while (!Serial);

  // Configure SPI
  mySPI.begin(MSBFIRST, SPI_MODE2);
  mySPI.onReceive(receiveEvent);
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
    mySPI.pushr(data);
    // Print data
    Serial.print(data, HEX);
  }
  Serial.println("]");
}
