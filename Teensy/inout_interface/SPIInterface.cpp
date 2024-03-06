#include "SPIInterface.h"

SPIInterface *init_spi_interface() {
    SPIInterface *spi_interface = (SPIInterface *) malloc(sizeof(SPIInterface));
    spi_interface->spi_slave =  new mySPI(0, SPI_8_BITS);
    spi->interface->i = 0;
    spi->interface->n = 0;

    mySPI.begin(MSBFIRST, SPI_MODE0);
    mySPI.swapPins();
    mySPI.onReceive(receiveEvent);
}

void spi_receive_event(SPIInterface *spi_interface) {
    SPISlave_T4 mySPI = spi_interface->spi_slave;

    //When there is data to read
    while ( mySPI.available() ) {

        uint32_t data = mySPI.popr();

        if (spi_interface->i == 0) {
            spi_interface->query = data;
            switch (data) {
                case QueryPositionControl :
                    spi_interface->n = 7;
                    break;

                case QuerySpeedControl:
                    spi_interface->n = 3;
                    break;

                case QueryIdle:
                    spi_interface->n = 1;
                    break;

                default: // Idle
                    spi_interface->n = 1;
                    break;
            }
        }
        // Get data
        spi_interface->data_buffer[spi_interface->i++] = data;

        #ifdef VERBOSE
        printf("NEW DATA : %d\n", (int) data);
        #endif
    }
}

#ifdef PARITY_CHECK
inline uint8_t parity_check(SPIInterface *spi_interface) {
    uint32_t n = spi_interface->n;
    uint8_t b = 0;
    for (int j = 0, j < n, j++) {
        b ^= __builtin_parity(data_buffer[j]);
    }
    return b;
} 
#endif