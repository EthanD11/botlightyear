#ifndef MECATROMINIBOT_UART_H
#define MECATROMINIBOT_UART_H

#include <lgpio.h>

/**
 * @brief Transfers data to UART device. Set one of the length to 0 to skip the step. gpioInitialise() must have been called before.
 * 
 * @param device The path to the device to communicate with, i.e. the virtual path "/dev/tty..." or "/dev/serial..."
 * @param baud The baud rate of the channel.
 * @param to_send The request, a string of chars (bytes) to send to the device
 * @param len_to_send The number of bytes to send (length of to_send)
 * @param to_receive A buffer to contain the answer of the device after the call.
 * @param len_to_receive The maximum number of bytes to read (the length of to_receive)
*/
void uart_xfer(char *device, unsigned int baud, char* to_send, unsigned int len_to_send, char* to_receive, unsigned int len_to_receive);

#endif