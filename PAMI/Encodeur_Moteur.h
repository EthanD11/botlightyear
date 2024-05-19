#ifndef ENCODEUR_MOTEUR_H
#define ENCODEUR_MOTEUR_H

#include "Global_Variable.h"

extern const int DroitA;
extern const int DroitB;
extern const int GaucheA;
extern const int GaucheB;

extern const int minSpeedPWM;
extern int maxSpeedPWM;

extern const float KI;
extern const float Kp;
extern const float Kd;

extern volatile float incSpeedDroit;
extern volatile float incSpeedGauche;

extern const float Kpmotor;
extern const float Kimotor;



extern volatile float intErrorSpeedGauche;
extern volatile float intErrorSpeedDroit;
extern float KpEncodeur;
extern float KdEncodeur;
extern float KIEncodeur;

//--------------------------------------------

#define PIN_ENCODEUR_GAUCHE_A 2 // GAUCHE
#define PIN_ENCODEUR_GAUCHE_B 7
#define PIN_ENCODEUR_DROIT_A 3// DROIT
#define PIN_ENCODEUR_DROIT_B 12

extern volatile long pasEncodeurGauche;
extern volatile long pasEncodeurDroit;
extern volatile int mouvement1;
extern volatile int mouvement2;
extern volatile bool calculXYneed;
extern const int seuilCalculDistance;
extern const int seuilPIDMotor;
extern volatile float RPMGauche;
extern volatile float RPMDroit;
extern volatile float lastTimeGauche;
extern volatile float lastTimeDroit;
extern volatile int pasEncodeurDroitPIDMotor;
extern volatile int pasEncodeurGauchePIDMotor;

int sign(double x);

void resetIntegrateurPIDMotor();
void initialisePID();
void motorDroit(MotorControl_t motorControl);
void motorGauche(MotorControl_t motorControl );

void updateEncoder();
void updateKpKiStart();
void updateKpKiPami3();
void updateKpKiPami2();
void updateKpKi();
//void EncoderGaucheChange();
//void EncoderDroitChange();
void calculXY();


#endif ENCODEUR_MOTEUR_H