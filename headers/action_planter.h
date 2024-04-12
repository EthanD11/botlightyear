#include "steppers.h"
#include "servos.h"
#include "teensy.h"
#include "action_variables.h"

extern Steppers steppers;
extern Flaps servo_flaps;
extern GripperDeployer deployer;
extern GripperHolder holder;
extern Teensy teensy;
extern ActionVariables actVar;

void planter_place_objects(uint8_t nObjects = 3);