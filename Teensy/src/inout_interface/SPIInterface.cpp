#include "SPIInterface.h"
#include "SPISlave_T4.h"

typedef struct SPIInterface {
    SPISlave_T4 *spi_slave;
    uint32_t i, n; // Number of bytes received, expected and actual
    uint32_t data_buffer[2*(2*256+1)+2]; // Max is obtained when 256 points are sent for path following
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
    int i = __spi_interface->i;
    uint32_t data;
    uint32_t *data_buffer = __spi_interface->data_buffer;

    //When there is data to read
    while ( mySPI->available() ) {
        data = mySPI->popr();
        data_buffer[i] = data;

        if (i == 0) __spi_interface->query = (query_t) data;
        switch(__spi_interface->query) {
            case QueryIdle:
                if (i == 0) __spi_interface->n = 1;
                break;
                
            case QueryDoPositionControl:
                if (i == 0) __spi_interface->n = 7;
                break;

            case QueryDoPathFollowing:
                if (i == 1) {
                    printf("QueryDoPathFollowing\n");
                    int ncheckpoints = (int) data;
                    __spi_interface->n = 2*(2*ncheckpoints+1) + 2;
                    printf("message expected size=%d\n", (int) __spi_interface->n);
                }
                break;

            default:
                break;
        }
        
        i += 1;

        #ifdef VERBOSE
        printf("NEW DATA : %d\n", (int) data);
        #endif
    }

    __spi_interface->i = i;
}

int spi_valid_transmission() {
    return (__spi_interface->i == __spi_interface->n);
}

void spi_reset_transmission() {
    __spi_interface->i = 0;
    __spi_interface->n = -1;
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
    int ncheckpoints = (int) __spi_interface->data_buffer[1];
    uint32_t *data = __spi_interface->data_buffer+2;
    
    double *x = (double *) malloc(sizeof(double)*ncheckpoints);
    double *y = (double *) malloc(sizeof(double)*ncheckpoints);

    char tmp_bytes[2];
    uint16_t tmp_16;
    for (int i = 0; i < ncheckpoints; i++) {
        tmp_bytes[0] = (char) data[2*i]; // First byte
        tmp_bytes[1] = (char) data[2*i+1]; // Second byte
        tmp_16 = *((uint16_t *) tmp_bytes); // Merge bytes
        x[i] = 2.0*(((double) tmp_16)/UINT16_MAX); // Decode
    }
    for (int i = 0; i < ncheckpoints; i++) {
        tmp_bytes[0] = (char) data[2*(i+ncheckpoints)]; // First byte
        tmp_bytes[1] = (char) data[2*(i+ncheckpoints)+1]; // Second byte
        tmp_16 = *((uint16_t *) tmp_bytes); // Merge bytes
        y[i] = 3.0*(((double) tmp_16)/UINT16_MAX); // Decode
    }
    tmp_bytes[0] = (char) data[4*ncheckpoints]; // First byte
    tmp_bytes[1] = (char) data[4*ncheckpoints+1]; // Second byte
    tmp_16 = *((uint16_t *) tmp_bytes); // Merge bytes
    double theta = (((double) tmp_16)/UINT16_MAX)*2*M_PI - M_PI; // Decode
    for (int i = 0; i < ncheckpoints; i++) {
        printf("(x[%d], y[%d]) = (%f,%f)\n", i, i, x[i], y[i]);
    }
    printf("theta0 = %f\n", theta);
    init_path_following(path_follower, x, y, ncheckpoints, theta);
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