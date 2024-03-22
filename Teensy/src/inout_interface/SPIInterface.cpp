#include "SPIInterface.h"
#include "SPISlave_T4.h"

typedef struct SPIInterface {
    SPISlave_T4 *spi_slave;
    uint32_t i, n; // Number of bytes received, expected and actual
    uint32_t data_buffer[2*(2*256+2)+2]; // Max is obtained when 256 points are sent for path following (plus 2 angles and 2 starting bytes)
    query_t query;

	char mode;
	double x, y, theta;

    double speed_refl, speed_refr;

    int dc_refl, dc_refr;
} SPIInterface;
void __spi_receive_event();

SPIInterface *__spi_interface;

void init_spi_interface() {
    __spi_interface = (SPIInterface *) malloc(sizeof(SPIInterface));
    __spi_interface->spi_slave =  new SPISlave_T4(0, SPI_8_BITS);
    __spi_interface->i = 0;
    __spi_interface->n = -1;
	__spi_interface->query = QueryIdle;

    __spi_interface->spi_slave->begin(MSBFIRST, SPI_MODE0);
    __spi_interface->spi_slave->swapPins();
    __spi_interface->spi_slave->onReceive(__spi_receive_event);
}

void spi_send_data(char mode) {
	__spi_interface->mode = mode;
}

query_t spi_get_query() {
    return __spi_interface->query;
}

void __spi_receive_event() {
    SPISlave_T4 *mySPI = __spi_interface->spi_slave;
	uint32_t data;
	uint32_t *data_buffer = __spi_interface->data_buffer;

	while (mySPI->available()) {
		int i = __spi_interface->i;
		data = mySPI->popr();
		data_buffer[i] = data;
		mySPI->pushr(data);
        
		if (i == 0) {
			__spi_interface->query = (query_t) data;
			printf("query = %d\n", (int) data);
			switch(__spi_interface->query) {
				case QueryIdle:
					__spi_interface->n = 1; // 1 of query
					break;
					
				case QueryDoPositionControl:
					__spi_interface->n = 7; // 1 of query, 6 of data
					break;

                case QueryDoSpeedControl:
                    __spi_interface->n = 5;
                    break;

				case QuerySetPosition:
					__spi_interface->n = 7; // 1 of query, 6 of data
					break;

                case QueryDoConstantDutyCycle:
                    __spi_interface->n = 5; // 1 of query, 2 of data
                    break;

				case QueryAskState:
                    // Not implemented
					break;

				case QueryDoPathFollowing:
					break;            

				default:
					break;
			}
		}
		else {
			switch(__spi_interface->query) {				
				case QueryAskState:
					// Not implemented
					break;

				case QueryDoPathFollowing:
					if (i == 1) {
						int ncheckpoints = (int) data;
						data = mySPI->popr(); 
						__spi_interface->n = 2*(2*ncheckpoints+2) + 2; // 1 of query, 1 of npoints, 2*npoints of 2 bytes, 2 theta of 2 bytes
					}
					#ifdef VERBOSE
					printf("QueryDoPathFollowing\n");
					printf("message expected size=%d\n", (int) __spi_interface->n);
					#endif
					break;            

				default:
					break;
			}
		}
		i += 1;
		__spi_interface->i = (uint32_t) i;
	}
}

int spi_valid_transmission() {
    return (__spi_interface->i == __spi_interface->n);
}

void spi_reset_transmission() {
    __spi_interface->i = 0;
    __spi_interface->n = -1;
}

void spi_handle_set_position(RobotPosition *rp) {
    uint32_t *data = __spi_interface->data_buffer+1; // skips query
    
    char two_bytes[2];
    uint16_t double_byte;

	two_bytes[0] = data[0];
    two_bytes[1] = data[1];
    double_byte = *((uint16_t *) two_bytes);
    rp->x = 2.0*((double) double_byte)/UINT16_MAX;
    
    two_bytes[0] = data[2];
    two_bytes[1] = data[3];
    double_byte = *((uint16_t *) two_bytes);
    rp->y = 3.0*((double) double_byte)/UINT16_MAX;

    two_bytes[0] = data[4];
    two_bytes[1] = data[5];
    double_byte = *((uint16_t *) two_bytes);
    rp->theta = (((double) double_byte)/UINT16_MAX)*2*M_PI - M_PI;

	printf("x = %f\ny = %f\ntheta = %f\n", rp->x, rp->y, rp->theta);
}

void spi_handle_position_control(PositionController *pc) 
{
    uint32_t *data = __spi_interface->data_buffer+1; // skips query
    
    char two_bytes[2];
    uint16_t double_byte;

    two_bytes[0] = data[0];
    two_bytes[1] = data[1];
    double_byte = *((uint16_t *) two_bytes);
    pc->xref      = 2.0*((double) double_byte)/UINT16_MAX;
    
    two_bytes[0] = data[2];
    two_bytes[1] = data[3];
    double_byte = *((uint16_t *) two_bytes);
    pc->yref      = 3.0*((double) double_byte)/UINT16_MAX;

    two_bytes[0] = data[4];
    two_bytes[1] = data[5];
    double_byte = *((uint16_t *) two_bytes);
    pc->theta_ref = (((double) double_byte)/UINT16_MAX)*2*M_PI - M_PI;

	printf("xref = %f\nyref = %f\ntheta_ref = %f\n", pc->xref, pc->yref, pc->theta_ref);
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
    double theta_start = (((double) tmp_16)/UINT16_MAX)*2*M_PI - M_PI; // Decode

    tmp_bytes[0] = (char) data[4*ncheckpoints+2]; // First byte
    tmp_bytes[1] = (char) data[4*ncheckpoints+3]; // Second byte
    tmp_16 = *((uint16_t *) tmp_bytes); // Merge bytes
    double theta_stop  = (((double) tmp_16)/UINT16_MAX)*2*M_PI - M_PI; // Decode


    for (int i = 0; i < ncheckpoints; i++) {
        printf("(x[%d], y[%d]) = (%f,%f)\n", i, i, x[i], y[i]);
    }
    printf("theta_start = %f\n", theta_start);
    printf("theta_end = %f\n", theta_stop);

    init_path_following(path_follower, x, y, ncheckpoints, theta_start, theta_stop);
    
    free(x);
    free(y);
}

void spi_handle_speed_control() {
    uint32_t *data = __spi_interface->data_buffer+1; // skips query
    
    char two_bytes[2];
    uint16_t double_byte;

    two_bytes[0] = data[0];
    two_bytes[1] = data[1];
    double_byte = *((uint16_t *) two_bytes);
    __spi_interface->speed_refl = 2.0*((double) double_byte)/UINT16_MAX;
    
    two_bytes[0] = data[2];
    two_bytes[1] = data[3];
    double_byte = *((uint16_t *) two_bytes);
    __spi_interface->speed_refr = 2.0*((double) double_byte)/UINT16_MAX;

	printf("speed_refl = %f\nspeed_refr = %f\n", __spi_interface->speed_refl, __spi_interface->speed_refr);
}

double spi_get_speed_refl() {
    return __spi_interface->speed_refl;
}
double spi_get_speed_refr() {
    return __spi_interface->speed_refr;
}


void spi_handle_constant_duty_cycle() {
    uint32_t *data = __spi_interface->data_buffer+1; // skips query
    
    char two_bytes[2];
    uint16_t double_byte;

    two_bytes[0] = data[0];
    two_bytes[1] = data[1];
    double_byte = *((uint16_t *) two_bytes);
    __spi_interface->dc_refl = (int) (((double) double_byte) - 255.0);
    
    two_bytes[0] = data[2];
    two_bytes[1] = data[3];
    double_byte = *((uint16_t *) two_bytes);
   __spi_interface->dc_refr = (int) (((double) double_byte) - 255.0);

	printf("dc_refl = %d\ndc_refr = %d\n", __spi_interface->dc_refl, __spi_interface->dc_refr);
}

double spi_get_dc_refl() {
    return __spi_interface->dc_refl;
}

double spi_get_dc_refr() {
    return __spi_interface->dc_refr;
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