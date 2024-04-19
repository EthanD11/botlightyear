#include "servos.h"

void Flaps::send_flaps_dutyCycle(uint16_t servo_flaps1_duty_cycle, uint16_t servo_flaps2_duty_cycle) {
    char send[5];
    send[0] = 0x80;
    
    send[1] = (servo_flaps1_duty_cycle & 0xFF00) >> 8;
    send[2] = servo_flaps1_duty_cycle & 0xFF;

    send[3] = (servo_flaps2_duty_cycle & 0xFF00) >> 8;
    send[4] = servo_flaps2_duty_cycle & 0xFF;
    this->bus->lock();
    this->bus->DE0_write(send);
    this->bus->unlock();
}



void Flaps::deploy() {
    this->send_flaps_dutyCycle(680,620);
}

void Flaps::raise() {
    this->send_flaps_dutyCycle(500,800);
}

void Flaps::idle() {
    this->send_flaps_dutyCycle(0,0);
}

void GripperDeployer::send_dutyCycle(uint16_t duty_cycle) {
    char send[5];
    send[0] = 0x8F;
    send[1] = 0; 
    send[2] = 0;
    send[3] = (duty_cycle & 0xFF00) >> 8;
    send[4] = duty_cycle & 0xFF;
    this->bus->lock();
    this->bus->DE0_write(send);
    this->bus->unlock();
}

void GripperDeployer::idle() {
    this->send_dutyCycle(0); 
}

void GripperDeployer::deploy() {
    this->send_dutyCycle(440); 
}

void GripperDeployer::half() {
    this->send_dutyCycle(650); 
}

void GripperDeployer::pot_deposit() {
    this->send_dutyCycle(650); 
}

void GripperDeployer::raise() {
    this->send_dutyCycle(850);  
}


void GripperHolder::send_dutyCycle(uint16_t duty_cycle) {
    char send[5];
    send[0] = 0x8E;
    send[1] = 0; 
    send[2] = 0;
    send[3] = (duty_cycle & 0xFF00) >> 8;
    send[4] = duty_cycle & 0xFF;
    this->bus->lock();
    this->bus->DE0_write(send);
    this->bus->unlock();
}

void GripperHolder::idle() {
    this->send_dutyCycle(0);
}
void GripperHolder::close() {
    this->send_dutyCycle(675);
} 
void GripperHolder::open() {
    this->send_dutyCycle(875);
} 

void GripperHolder::open_full() {
    this->send_dutyCycle(1100);
}
void GripperHolder::hold_pot() {
    this->send_dutyCycle(725);
} 
void GripperHolder::hold_plant() {
    this->send_dutyCycle(645);
}