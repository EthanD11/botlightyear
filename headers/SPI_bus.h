#ifndef _BLY_SPI_BUS_H_
#define _BLY_SPI_BUS_H_

class SPIBus
{
private:
    int DE0_handle, Teensy_handle;
public:
    SPIBus();
    ~SPIBus();
    int test();
    void lock();
    void unlock();
    void DE0_write(char *send);
    void DE0_xfer(char *send, char *receive);
    void Teensy_write(char *send, int msgSize);
    void Teensy_xfer(char *send, char *receive, int msgSize);
};

class SPIUser
{
public:
    SPIBus *bus;
    SPIUser(SPIBus *bus);
    ~SPIUser();
}; 

SPIUser::SPIUser(SPIBus *bus) {
    this->bus = bus;
}

SPIUser::~SPIUser() {}; 
#endif