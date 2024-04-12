#include "shared_variables.h"
#include <pthread.h>

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
    teensy.idle();
    steppers.reset_all();
    servoFlaps.idle(); grpDeployer.idle(); grpHolder.idle();

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
    time(&tStart);
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

void SharedVariables::get_adv_pos(double *xAdv, double *yAdv, double *thetaAdv) {
    pthread_rwlock_rdlock(&advPosLock);
    *xAdv = this->xAdv; *yAdv = this->yAdv; *thetaAdv = this->thetaAdv;
    pthread_rwlock_unlock(&advPosLock);
}

void SharedVariables::set_adv_pos(double xAdv, double yAdv, double thetaAdv) {
    pthread_rwlock_wrlock(&advPosLock);
    this->xAdv = xAdv; this->yAdv = yAdv; this->thetaAdv = thetaAdv;
    pthread_rwlock_unlock(&advPosLock);
}
