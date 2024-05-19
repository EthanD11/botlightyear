#include "shared_variables.h"
#include <pthread.h>
#include <unistd.h>
#include <stdio.h>

//#define VERBOSE

static pthread_rwlock_t robotPosLock, advPosLock;

SharedVariables::SharedVariables()
{
    if (pthread_rwlock_init(&robotPosLock, NULL) != 0) exit(1);
    if (pthread_rwlock_init(&advPosLock, NULL) != 0) exit(1);

    color = NoTeam;
    score = 0;
    for (uint8_t i = 0; i < 8; i++) storage[i] = ContainsNothing;
    for (uint8_t i = 0; i < 2; i++) SPsDone[i] = 0;
    for (uint8_t i = 0; i < 6; i++) plantCounts[i] = 6; 
    bulldoDone = 0;
    
    for (uint8_t j = 0; j<3; j++) {
        plantersDone[j] = 0; 
        zonesDone[j] = 0;
    }
    for (uint8_t j=0; j<5; j++) valids[j] = 0; 
    nFreeSlots = 8;
    spBlockDone = 0; 
    backToBaseDone = 0;
    goingToBase = 0;

    xAdv = 400; yAdv = 0; dAdv = 400; aAdv = 0;
    abort_time = 20; 
    spiBus = new SPIBus();
    pins = new GPIOPins();
    teensy = new Teensy(spiBus, pins);
    odo = new Odometry(spiBus);
    servoFlaps = new Flaps(spiBus);
    grpDeployer = new GripperDeployer(spiBus);
    grpHolder = new GripperHolder(spiBus);
    steppers = new Steppers(spiBus, pins);
    graph = new Graph();
}

SharedVariables::~SharedVariables()
{

    delete graph;
    delete spiBus;
    delete pins;

    steppers->bus = NULL; steppers->pins = NULL; delete steppers;
    teensy->bus = NULL; teensy->pins = NULL; delete teensy;
    servoFlaps->bus = NULL; delete servoFlaps;
    grpDeployer->bus = NULL; delete grpDeployer;
    grpHolder->bus = NULL; delete grpHolder;

    pthread_rwlock_destroy(&robotPosLock);
    pthread_rwlock_destroy(&advPosLock);

}

void SharedVariables::start_timer() {
    #ifdef VERBOSE
    printf("Waiting for starting cord setup... \n");
    #endif

    pins->wait_for_gpio_value(StartingCordGPIO, 0, 2000000);

    #ifdef VERBOSE
    printf("Starting cord has been setup\n");
    #endif
    usleep(1000000);

    fprintf(stderr,"Waiting start of the game... \n");

    pins->wait_for_gpio_value(StartingCordGPIO, 1, 2000000);
    time(&tStart);

    printf("Game started! \n");
}

int16_t SharedVariables::update_and_get_timer() {
    return (int16_t) (TOTAL_GAME_TIME - (time(NULL) - tStart));
}   

void SharedVariables::get_robot_pos(double *x, double *y, double *theta) {
    pthread_rwlock_rdlock(&robotPosLock);
    if (x != NULL) *x = this->x;
    // else printf("x is NULL\n");
    if (y != NULL) *y = this->y;
    // else printf("y is NULL\n");
    if (theta != NULL) *theta = this->theta;
    // else printf("theta is NULL\n");
    pthread_rwlock_unlock(&robotPosLock);
}

void SharedVariables::set_robot_pos(double x, double y, double theta) {
    pthread_rwlock_wrlock(&robotPosLock);
    this->x = x; this->y = y; this->theta = theta;
    pthread_rwlock_unlock(&robotPosLock);
}

void SharedVariables::get_adv_pos(double *xAdv, double *yAdv, double *dAdv, double *aAdv) {
    pthread_rwlock_rdlock(&advPosLock);
    if (xAdv != NULL) *xAdv = this->xAdv;
    if (yAdv != NULL) *yAdv = this->yAdv;
    if (dAdv != NULL) *dAdv = this->dAdv;
    if (aAdv != NULL) *aAdv = this->aAdv;
    pthread_rwlock_unlock(&advPosLock);
}

void SharedVariables::set_adv_pos(double xAdv, double yAdv, double dAdv, double aAdv) {
    pthread_rwlock_wrlock(&advPosLock);
    this->xAdv = xAdv; this->yAdv = yAdv; this->dAdv = dAdv; this->aAdv = aAdv;
    pthread_rwlock_unlock(&advPosLock);
}

void SharedVariables::teensy_reset_pos() {
    double x, y, theta;
    this->odo->get_pos(&x, &y, &theta);
    this->teensy->set_position(x, y, theta);

}