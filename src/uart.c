#include "uart.h"

void uart_xfer(char *device, unsigned int baud, char* to_send, unsigned int len_to_send, char* to_receive, unsigned int len_to_receive) {

    int handle;
    handle = serOpen(device, baud, 0);
    char trash[256];
    serRead(handle, trash, 256);
    if (len_to_send > 0) serWrite(handle, to_send, len_to_send);
    while (serDataAvailable(handle) < len_to_receive) gpioDelay(1000); 
    serRead(handle, to_receive, len_to_receive);
    serRead(handle, trash, 256);
    serClose(handle);

}