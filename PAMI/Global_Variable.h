#ifndef GLOBAL_VARIABLE_H
#define GLOBAL_VARIABLE_H

#include <math.h>
#include <ezButton.h>
#include <HCSR04.h>
#include <Servo.h>
#include <PID_v1.h>

//parametre du robot
// Diamètre des roues en mm
#define RAYON_ROUE 29.25
// Distance entre les roues en mm
#define DISTANCE_ENTRE_ROUES 94.5 // avant 97
// Ratio de réduction des encodeurs
#define PULS_TOUR_ROUE 1536//8par tour avr reduction 48 = 384* 4 car librairy encodeur
// 960 //(8 pulse par tour soit 960 par tour de roue (gearRatio=120))

#define PosInitialeX_PAMI1  96 
#define PosInitialeY_PAMI1  225

#define PosInitialeX_PAMI2  96 
#define PosInitialeY_PAMI2  400

#define PosInitialeX_PAMI3 96
#define PosInitialeY_PAMI3 75

#define AngleInitiale  0

extern int PAMI;

extern int TARGET_X_PAMI1;
extern int TARGET_X_PAMI1_BLEU;
extern int TARGET_XPRIME_PAMI1_BLEU;
extern int TARGET_Y_PAMI1;
extern int LIMITE_Y_ZONE_PAMI1;
extern int LIMITE_Y_STOP_ZONE_PAMI1;

extern int TARGET_X_PAMI2;
extern int TARGET_Y_PAMI2;
extern int TARGET_XPRIME_PAMI2;
extern int TARGET_YPRIME_PAMI2;
extern int TARGET_YPRIME_BLUE_PAMI2; //BLUE
extern int TARGET_X_IN_ZONE_PAMI2;
extern int TARGET_Y_IN_ZONE_PAMI2;
extern int TARGET_Y_IN_ZONE_BLUE_PAMI2; //BLUE
//extern int LIMITE_Y_ZONE_PAMI2;
extern int LIMITE_X_ZONE_PAMI2;
extern int LIMITE_Y_STOP_ZONE_PAMI2;
extern int LIMITE_X_STOP_ZONE_PAMI2;

extern int TARGET_X_PAMI3;
extern int TARGET_Y_PAMI3 ;
extern int TARGET_XPRIME_PAMI3 ;
extern int TARGET_YPRIME_PAMI3 ;
extern int LIMITE_Y_ZONE_PAMI3 ;
extern int LIMITE_X_ZONE_PAMI3 ;
extern int LIMITE_Y_STOP_ZONE_PAMI3 ;
extern int LIMITE_X_STOP_ZONE_PAMI3;

extern String COLOR;

typedef struct PositionRobot {
  volatile float angleRotation;// Angle de rotation EN RADIAN
  volatile float positionX;
  volatile float positionY;
} PositionRobot_t; 
inline PositionRobot_t positionRobot;//  = {AngleInitiale,PosInitialeX,PosInitialeY};

typedef struct MotorControl{
  float speedDroit;
  float speedGauche;
  char brakeOrAccDroite;
  char brakeOrAccGauche;
  
  float IntErrorDroit;
  float IntErrorGauche;
  float derivatieError;
} MotorControl_t; 
inline MotorControl_t motorControl = {0, 0, 'F', 'F', 0, 0, 0};



extern int maxSpeed;
extern int normalSpeed;
extern int normalSpeedInZone;
extern int maxSpeedInZone;
extern const int maxSpeedSearch;
extern int errorPrevious;



void initialise();

#endif GLOBAL_VARIABLE_H