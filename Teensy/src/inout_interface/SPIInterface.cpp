#include "SPIInterface.h"

typedef struct SPIInterface {
    SPISlave_T4 spi_slave;
    uint32_t i, n; // Number of bytes received, expected and actual
    uint32_t data_buffer[7];
    query_t query;
} SPIInterface;

SPIInterface *spi_interface;

void init_spi_interface() {
    spi_interface = (SPIInterface *) malloc(sizeof(SPIInterface));
    spi_interface->spi_slave =  new mySPI(0, SPI_8_BITS);
    spi->interface->i = 0;
    spi->interface->n = 0;

    mySPI.begin(MSBFIRST, SPI_MODE0);
    mySPI.swapPins();
    mySPI.onReceive(spi_receive_event);
}

query_t spi_get_query() {
    return spi_interface->query;
}

void spi_receive_event() {
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

int spi_valid_transmission() {
    return (spi_interface->i == spi_interface->n)
}

void spi_reset_transmission() {
    spi_interface->i = 0;
}

void spi_handle_position_control(RobotPosition *rp, PositionController *pc) 
{
    uint32_t data_buffer = spi_interface->data_buffer;
    rp->x         = ((double)(dataBuf[1]))*3/255;
    rp->y         = ((double)(dataBuf[2]))*2/255;
    rp->theta     = ((double)(dataBuf[3]))*2*M_PI/255 - M_PI;
    pc->xref      = ((double)(dataBuf[4]))*3/255;
    pc->yref      = ((double)(dataBuf[5]))*2/255;
    pc->theta_ref = ((double)(dataBuf[6]))*2*M_PI/255 - M_PI;
}

void spi_handle_speed_control(Regulator *speed_regulator) {
    uint32_t data_buffer = spi_interface->data_buffer;
    speed_regulator->speed_refl = ((double)data_buffer[1])*2/255;
    speed_regulator->speed_refr = ((double)data_buffer[2])*2/255;
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