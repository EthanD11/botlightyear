#include "uart.h"

void uart_xfer(char *device, unsigned int baud, char* to_send, unsigned int len_to_send, char* to_receive, unsigned int len_to_receive) {

    int handle;
    handle = lgSerialOpen(device, baud, 0);
    char trash[256];
    lgSerialRead(handle, trash, 256);
    if (len_to_send > 0) lgSerialWrite(handle, to_send, len_to_send);
    while (lgSerialDataAvailable(handle) < len_to_receive) lguSleep(1e-3); 
    lgSerialRead(handle, to_receive, len_to_receive);
    lgSerialRead(handle, trash, 256);
    lgSerialClose(handle);

}