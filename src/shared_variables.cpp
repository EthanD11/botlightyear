#include "shared_variables.h"
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>

//#define VERBOSE

pthread_rwlock_t robotPosLock, advPosLock;

SharedVariables::SharedVariables()
{
    pthread_rwlock_init(&robotPosLock, NULL);
    pthread_rwlock_init(&advPosLock, NULL);

    color = NoTeam;
    score = 0;
    for (uint8_t i = 0; i < 8; i++) storage[i] = ContainsNothing;
    nFreeSlots = 8;
}

SharedVariables::~SharedVariables()
{

    graph.~Graph();
    spiBus.~SPIBus();
    pins.~GPIOPins();

    steppers.bus = NULL; steppers.pins = NULL; steppers.~Steppers();
    teensy.bus = NULL; teensy.pins = NULL; teensy.~Teensy();
    servoFlaps.bus = NULL; servoFlaps.~Flaps();
    grpDeployer.bus = NULL; grpDeployer.~GripperDeployer();
    grpHolder.bus = NULL; grpHolder.~GripperHolder();

    pthread_rwlock_destroy(&robotPosLock);
    pthread_rwlock_destroy(&advPosLock);

}

void SharedVariables::start_timer() {
    #ifdef VERBOSE
    printf("Waiting for starting cord setup... \n");
    #endif

    pins.wait_for_gpio_value(StartingCordGPIO, 0, 2000000);

    #ifdef VERBOSE
    printf("Starting cord has been setup\n");
    #endif
    usleep(1000000);

    #ifdef VERBOSE
    printf("Waiting start of the game... \n");
    #endif

    pins.wait_for_gpio_value(StartingCordGPIO, 1, 2000000);
    time(&tStart);

    #ifdef VERBOSE
    printf("Game started! \n");
    #endif
}

int8_t SharedVariables::update_and_get_timer() {
    return (int8_t) (100 - (time(NULL) - tStart));
}

void SharedVariables::get_robot_pos(double *x, double *y, double *theta) {
    pthread_rwlock_rdlock(&robotPosLock);
    *x = this->x; *y = this->y; *theta = this->theta;
    pthread_rwlock_unlock(&robotPosLock);
}

void SharedVariables::set_robot_pos(double x, double y, double theta) {
    pthread_rwlock_wrlock(&robotPosLock);
    this->x = x; this->y = y; this->theta = theta;
    pthread_rwlock_unlock(&robotPosLock);
}

void SharedVariables::get_adv_pos(double *xAdv, double *yAdv) {
    pthread_rwlock_rdlock(&advPosLock);
    *xAdv = this->xAdv; *yAdv = this->yAdv;
    pthread_rwlock_unlock(&advPosLock);
}

void SharedVariables::set_adv_pos(double xAdv, double yAdv) {
    pthread_rwlock_wrlock(&advPosLock);
    this->xAdv = xAdv; this->yAdv = yAdv;
    pthread_rwlock_unlock(&advPosLock);
}
