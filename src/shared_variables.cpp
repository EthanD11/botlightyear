#include "action_variables.h"
#include <pthread.h>

pthread_rwlock_t var_mutex;

ActionVariables::ActionVariables()
{
    pthread_rwlock_init(&var_mutex, NULL);
    nextFreeSlot = SlotM3;
}

ActionVariables::~ActionVariables()
{
    pthread_rwlock_destroy(&var_mutex);
}

void ActionVariables::pos_rd_lock() {
    pthread_rwlock_rdlock(&var_mutex);
}

void ActionVariables::pos_wr_lock() {
    pthread_rwlock_wrlock(&var_mutex);
}

void ActionVariables::pos_unlock() {
    pthread_rwlock_unlock(&var_mutex);
}