#ifndef _SPIINTERFACE_H_
#define _SPIINTERFACE_H_

#include "utils.h"
#include "SPISlave_T4.h"

#ifdef PARITY_CHECK
uint8_t parity_bit;
#endif


typedef enum {
  QueryIdle, // Idle, reset motor voltages to 0V
  QueryTestRead, // SPI test, answer with [1,2,3,4]
  QueryTestWrite, // SPI test, answer with data received
  QueryPositionControl, // Position update, data received = [flag,x,y,t,xr,yr,tr]
  QuerySpeedControl // Speed control, data received = [flag,left,right]
} query_t;

typedef struct SPIInterface {
    SPISlave_T4 spi_slave;
    uint32_t i, n; // Number of bytes received, expected and actual
    uint32_t data_buffer[7];
    query_t query;
} SPIInterface;

SPIInterface *init_spi_interface();
void spi_receive_event(SPIInterface *spi_interface);

#endif