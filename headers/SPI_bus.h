#ifndef _BLY_SPI_BUS_H_
#define _BLY_SPI_BUS_H_
#include <cstddef>

class SPIBus
{
private:
    int DE0_handle, Teensy_handle;
public:
    /**
     * @brief The SPI Bus Object. Handles all the communications between the RPI and the two slaves : DE0 and Teensy
     */  
    SPIBus();
    ~SPIBus();
    int test();
    /**
     * @brief Locks the bus' mutex
     */  
    void lock();
    /**
     * @brief Unlocks the bus' mutex
     */  
    void unlock();
    /**
     * @brief Write to the DE0
     * @param send A communication to send to the DE0. Format : 5 chars, 1 for request and 4 for data
     */ 
    void DE0_write(char *send);
    /**
     * @brief Write to the DE0 and receive the response
     * @param send A communication to send to the DE0. Format : 5 chars, 1 for request and 4 for data
     * @param receive A pointer to retrieve the communication send back by the DE0
     */ 
    void DE0_xfer(char *send, char *receive);
    /**
     * @brief Write to the Teensy
     * @param send A communication to send to the teensy
     * @param msgSize The size of the communication (in bytes)
     */ 
    void Teensy_write(char *send, int msgSize);
    /**
     * @brief Write to the Teensy and receive the response
     * @param send A communication to send to the teensy
     * @param receive A pointer to retrieve the communication send back by the teensy
     * @param msgSize The size of the communication (in bytes)
     */ 
    void Teensy_xfer(char *send, char *receive, int msgSize);
};

class SPIUser
{
public:
    SPIBus *bus = NULL;
    /*
    Class defining all the SPI users' methods
    When using the SPI, a class needs to extend this one
    */
    SPIUser(SPIBus *bus);
    ~SPIUser();
};

#endif