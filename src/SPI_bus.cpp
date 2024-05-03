#include "SPI_bus.h"
#include <lgpio.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>

//#define VERBOSE

#define SPI_CLK_FREQ 1000000
#define SPI_MODE_DEFAULT 0

pthread_mutex_t mutex;

SPIBus::SPIBus() {
    #ifdef VERBOSE
    printf("Constructor SPI Bus\n");
    #endif
    DE0_handle = lgSpiOpen(0, 1, SPI_CLK_FREQ, SPI_MODE_DEFAULT);
    Teensy_handle = lgSpiOpen(0, 0, SPI_CLK_FREQ, SPI_MODE_DEFAULT);
    if (pthread_mutex_init(&mutex,NULL) != 0 || DE0_handle < 0 || Teensy_handle < 0) exit(-1);
    int test_res = test();
    if (test_res != 0) exit(test_res);
}

SPIBus::~SPIBus() {
    #ifdef VERBOSE
    printf("Destructor SPI Bus\n");
    #endif
    lgSpiClose(DE0_handle);
    lgSpiClose(Teensy_handle);
    pthread_mutex_destroy(&mutex);
}

int SPIBus::test() {
    char send1[5] = {0,0,0,0,0}; send1[0] = 0; // Test read
    char send2[] = {0x9F,0x05,0x04,0x03,0x02}; // Test write
    char send3[5] = {0x1F,0,0,0,0}; send3[0] = 0x1F;
    char receive1[5] = {0,0,0,0,0}; char receive3[5] = {0,0,0,0,0};

    pthread_mutex_lock(&mutex);
    lgSpiXfer(DE0_handle, send1, receive1, 5);
    lgSpiWrite(DE0_handle, send2, 5);
    lgSpiXfer(DE0_handle, send3, receive3, 5);
    pthread_mutex_unlock(&mutex);

    uint8_t failure = 0;
    for (int i = 0; i < 5; i++)
    {
        if (receive1[i] != i) {
            printf("SPI test 1 failed : receive[%d] == %d != %d\n",i,receive1[i],i);
            failure = 1;
        }
    }
    if (receive3[0] != 0x00) {printf("SPI test 2 failed : receive1[%d] == %d != %d\n",0,receive3[0],0x00); failure |= 2;}
    if (receive3[1] != 0x05) {printf("SPI test 2 failed : receive1[%d] == %d != %d\n",1,receive3[1],0x05); failure |= 2;}
    if (receive3[2] != 0x04) {printf("SPI test 2 failed : receive1[%d] == %d != %d\n",2,receive3[2],0x04); failure |= 2;}
    if (receive3[3] != 0x03) {printf("SPI test 2 failed : receive1[%d] == %d != %d\n",3,receive3[3],0x03); failure |= 2;}
    if (receive3[4] != 0x02) {printf("SPI test 2 failed : receive1[%d] == %d != %d\n",4,receive3[4],0x02); failure |= 2;}
    return failure;
}

void SPIBus::lock() {
    pthread_mutex_lock(&mutex);
}

void SPIBus::unlock() {
    // usleep(50000);
    pthread_mutex_unlock(&mutex);
}

void SPIBus::DE0_write(char *send) {
    if (lgSpiWrite(DE0_handle, send, 5) < 0) printf("SPI DE0 Write failed\n");
}

void SPIBus::DE0_xfer(char *send, char *receive) {
     if (lgSpiXfer(DE0_handle, send, receive, 5) < 0) printf("SPI DE0 Xfer failed\n");
}

void SPIBus::Teensy_write(char *send, int msgSize) {
     if (lgSpiWrite(Teensy_handle, send, msgSize) < 0) printf("SPI Teensy Write failed\n");
}

void SPIBus::Teensy_xfer(char *send, char *receive, int msgSize) {
     if (lgSpiXfer(Teensy_handle, send, receive, msgSize) < 0) printf("SPI Teensy Xfer failed\n");
}

SPIUser::SPIUser(SPIBus *bus) {
    #ifdef VERBOSE
    printf("Constructor SPI user\n");
    #endif
    this->bus = bus;
}

SPIUser::~SPIUser() {
    #ifdef VERBOSE
    printf("Destructor SPI user\n");
    #endif
    if (bus == NULL) return;
    bus->~SPIBus();
    bus = NULL;
}; 