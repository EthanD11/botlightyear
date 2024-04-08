#include "SPI_Modules.h"

int main(int argc, char const *argv[])
{
    if (init_spi() != 0) return -1;
    if (test_spi() != 0) exit(2);

    /*stpr_reset_all();
    lguSleep(1);
    stpr_calibrate_all();*/

    /*plate_move(1);
    lguSleep(3);
    stpr_reset_all();*/

    /*flaps_servo_cmd(FlapsDeploy);
    lguSleep(3);
    flaps_servo_cmd(FlapsRaise);
    lguSleep(1);
    flaps_servo_cmd(FlapsIdle);*/
    
    /*gripper_holder_cmd(HolderClosed);
    lguSleep(3);
    gripper_holder_cmd(HolderOpen);
    lguSleep(3);
    gripper_holder_cmd(HolderPlant);
    lguSleep(3);
    gripper_holder_cmd(HolderPot);
    lguSleep(1);
    gripper_holder_cmd(HolderIdle);*/

    //gripper_deployer_cmd(DeployerDeploy);
    //lguSleep(3);
    //gripper_deployer_cmd(DeployerHalf);
    //lguSleep(3);
    gripper_deployer_cmd(DeployerRaise);
    //lguSleep(1);
    //gripper_holder_cmd(HolderIdle);

    close_spi();
    return 0;
}
