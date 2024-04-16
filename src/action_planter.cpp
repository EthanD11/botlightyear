#include "action_planter.h"
#include "shared_variables.h"
#include <pthread.h>

pthread_t KCID;
volatile uint8_t leaveFlag = 0, atDestFlag = 0;

void leave() {
    leaveFlag = 1;
    pthread_join(KCID, NULL);
}

void *kinematic_chain(void *args) {

    return NULL;
}

void ActionPlanter::do_action() {
    pthread_create(&KCID, NULL, kinematic_chain, NULL);
    if (path_following_to_action(path)) return leave();

    atDestFlag = 1;
    action_position_control()
}