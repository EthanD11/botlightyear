#include "SPIInterface.h"
#include "SPISlave_T4.h"

typedef struct SPIInterface {
    SPISlave_T4 *spi_slave;
    uint32_t i, n; // Number of bytes received, expected and actual
    uint32_t data_buffer[7];
    query_t query;
} SPIInterface;
void __spi_receive_event();

SPIInterface *__spi_interface;

void init_spi_interface() {
    __spi_interface = (SPIInterface *) malloc(sizeof(SPIInterface));
    __spi_interface->spi_slave =  new SPISlave_T4(0, SPI_8_BITS);
    __spi_interface->i = 0;
    __spi_interface->n = -1;

    __spi_interface->spi_slave->begin(MSBFIRST, SPI_MODE0);
    __spi_interface->spi_slave->swapPins();
    __spi_interface->spi_slave->onReceive(__spi_receive_event);
}

query_t spi_get_query() {
    return __spi_interface->query;
}

void __spi_receive_event() {
    SPISlave_T4 *mySPI = __spi_interface->spi_slave;

    //When there is data to read
    while ( mySPI->available() ) {

        uint32_t data = mySPI->popr();

        if (__spi_interface->i == 0) {
            __spi_interface->query = (query_t) data;
            switch (data) {
                case QueryDoPositionControl :
                    __spi_interface->n = 7;
                    break;

                case QueryIdle:
                    __spi_interface->n = 1;
                    break;

                default: // Idle
                    __spi_interface->n = 1;
                    break;
            }
        }
        // Get data
        __spi_interface->data_buffer[__spi_interface->i++] = data;

        #ifdef VERBOSE
        printf("NEW DATA : %d\n", (int) data);
        #endif
    }
}

int spi_valid_transmission() {
    return (__spi_interface->i == __spi_interface->n);
}

void spi_reset_transmission() {
    __spi_interface->i = 0;
}

void spi_handle_position_control(RobotPosition *rp, PositionController *pc) 
{
    uint32_t *data_buffer = __spi_interface->data_buffer;
    rp->x         = ((double)(data_buffer[1]))*3/255;
    rp->y         = ((double)(data_buffer[2]))*2/255;
    rp->theta     = ((double)(data_buffer[3]))*2*M_PI/255 - M_PI;
    pc->xref      = ((double)(data_buffer[4]))*3/255;
    pc->yref      = ((double)(data_buffer[5]))*2/255;
    pc->theta_ref = ((double)(data_buffer[6]))*2*M_PI/255 - M_PI;
}


void spi_handle_path_following(PathFollower *path_follower) {

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