#include "Global_Variable.h"

int PAMI = 3;

// Rex --> 1 Jardiniere DOUBLE
// Woody --> 2 zone loin:-)
// Jessie --> 3 Jardiniere SIMPLE POSITION 3

int TARGET_X_PAMI1 = 650;
int TARGET_X_PAMI1_BLEU = 493; // 493;
int TARGET_XPRIME_PAMI1_BLEU = 400;//550
int TARGET_Y_PAMI1 = 1800;
int LIMITE_Y_ZONE_PAMI1 = 1000;
int LIMITE_Y_STOP_ZONE_PAMI1 = 1500;

int TARGET_X_PAMI2 = 550;
int TARGET_Y_PAMI2 = 700;
int TARGET_XPRIME_PAMI2 = 1550;
int TARGET_YPRIME_PAMI2 = 1000;
int TARGET_YPRIME_BLUE_PAMI2 = 1200;
int TARGET_X_IN_ZONE_PAMI2 = 2100;
int TARGET_Y_IN_ZONE_PAMI2 = 1600;
int TARGET_Y_IN_ZONE_BLUE_PAMI2 =  2000;
int LIMITE_X_ZONE_PAMI2 = 1400;
//int LIMITE_Y_ZONE_PAMI2 = 900;
int LIMITE_X_STOP_ZONE_PAMI2 = 2000;
int LIMITE_Y_STOP_ZONE_PAMI2 = 1500;


int TARGET_X_PAMI3 = 350;
int TARGET_Y_PAMI3 = 600;
int TARGET_XPRIME_PAMI3 = -10;
int TARGET_YPRIME_PAMI3 = 850;
int LIMITE_X_ZONE_PAMI3 = 100;
int LIMITE_Y_ZONE_PAMI3 = 550;
int LIMITE_X_STOP_ZONE_PAMI3 = 0;
int LIMITE_Y_STOP_ZONE_PAMI3 = 600;


String COLOR = "BLUE";

/*volatile float angleRotation = AngleInitiale; // Angle de rotation EN RADIAN
volatile float positionX = PosInitialeX;
volatile float positionY = PosInitialeY;*/
//positionRobot = {AngleInitiale,PosInitialeX,PosInitialeY};

/*int speedDroit = 0;
int speedGauche = 0;
char brakeOrAccDroite = 'F';
char brakeOrAccGauche = 'F';
float IntErrorDroit = 0;
float IntErrorGauche = 0;
float derivatieError = 0;*/
//motorControl = {0, 0, 'F', 'F', 0, 0, 0};

int maxSpeed = 1100;
int normalSpeed = 700;
int normalSpeedInZone = 300;
int maxSpeedInZone = 700;
const int maxSpeedSearch = 150;
int errorPrevious = 0;

