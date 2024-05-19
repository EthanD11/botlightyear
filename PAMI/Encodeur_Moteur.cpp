#include "Encodeur_Moteur.h"
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>


const int DroitA = 8;
const int DroitB = 11;
const int GaucheA = 4;
const int GaucheB = 5;

const int minSpeedPWM = 25;//75
int maxSpeedPWM = 200;

const float KI = 0.001;
const float Kp = 4;
const float Kd = 0;

volatile float incSpeedDroit = 0;
volatile float incSpeedGauche = 0;

const float KpmotorDroitOriginal = 0.5;
const float KimotorDroitOriginal = 0.3;
const float KpmotorGaucheOriginal = 0.5;
const float KimotorGaucheOriginal = 0.3;

const float KpmotorDroitStart = 0.2;
const float KimotorDroitStart = 0.3;
const float KpmotorGaucheStart = 10.5;
const float KimotorGaucheStart = 0.3;

const float KpmotorDroitPAMI2 = 0.5;
const float KimotorDroitPAMI2 = 0.3;
const float KpmotorGauchePAMI2 = 0.5;
const float KimotorGauchePAMI2 = 0.3;

const float KpmotorDroitPAMI3 = 0.5;
const float KimotorDroitPAMI3 = 0.3;
const float KpmotorGauchePAMI3 = 0.5;
const float KimotorGauchePAMI3 = 0.3;




/*float KpmotorDroit = KpmotorDroitStart;
float KimotorDroit = KimotorDroitStart;
float KpmotorGauche = KpmotorGaucheStart;
float KimotorGauche = KimotorGaucheStart;*/

float KpmotorDroit = KpmotorDroitOriginal;
float KimotorDroit = KimotorDroitOriginal;
float KpmotorGauche = KpmotorGaucheOriginal;
float KimotorGauche = KimotorGaucheOriginal;

volatile float intErrorSpeedGauche = 0;
volatile float intErrorSpeedDroit = 0;
float KpEncodeur = 140;//100
float KdEncodeur = 2;
float KIEncodeur = 10;//10

double SetpointMotorDroit, InputMotorDroit, OutputMotorDroit;
PID PIDMotorDroit(&InputMotorDroit, &OutputMotorDroit, &SetpointMotorDroit, KpmotorDroit, KimotorDroit, 0, DIRECT);
double SetpointMotorGauche, InputMotorGauche, OutputMotorGauche;
PID PIDMotorGauche(&InputMotorGauche, &OutputMotorGauche, &SetpointMotorGauche, KpmotorGauche, KimotorGauche, 0, DIRECT);


//-----------------------------------------------------

// declaration pour encodeur
// Compteurs pour les impulsions des encodeurs
volatile long pasEncodeurGauche = 0; // volatile car peut etre changer par une interruption de programme
volatile long pasEncodeurDroit = 0;
// Position du robot sur le plateau
volatile int mouvement1 =0;
volatile int mouvement2 = 0;
volatile bool calculXYneed = false;
const int seuilCalculDistance = 2; // tout les x tic d encodeur on recalcule la position x,y;
const int seuilPIDMotor = 8*4;
volatile float RPMGauche = 0;
volatile float RPMDroit =0;
volatile float lastTimeGauche = 0;
volatile float lastTimeDroit = 0;
volatile int pasEncodeurDroitPIDMotor = 0;
volatile int pasEncodeurGauchePIDMotor =0;
// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder encodeurGauche(PIN_ENCODEUR_GAUCHE_A, PIN_ENCODEUR_GAUCHE_B);
Encoder encodeurDroit(PIN_ENCODEUR_DROIT_A, PIN_ENCODEUR_DROIT_B);




int sign(double x) {
    //fonction sign mathematique
    return (x > 0) - (x < 0);
}

void resetIntegrateurPIDMotor(){
  // fonction pour reset l'integrateur du PID
  PIDMotorDroit.SetMode(MANUAL);//reset de l'integrateur moteur droit
  OutputMotorDroit = 0;
  PIDMotorDroit.SetMode(AUTOMATIC);
  PIDMotorGauche.SetMode(MANUAL);//reset de l'integrateur moteur gauche
  OutputMotorGauche = 0;
  PIDMotorGauche.SetMode(AUTOMATIC);
  encodeurGauche.write(0);
  encodeurDroit.write(0);
}

void initialisePID(){
  // fonction pour initialiser les PID
  PIDMotorDroit.SetMode(AUTOMATIC);
  PIDMotorDroit.SetOutputLimits(0,maxSpeedPWM);
  PIDMotorGauche.SetMode(AUTOMATIC);
  PIDMotorGauche.SetOutputLimits(0,maxSpeedPWM);
}

void updateKpKiPami2(){
  // fonction pour changer les Kp et Ki du PID du PAMI 2
  KpmotorDroit = KpmotorDroitPAMI2;
  KimotorDroit = KimotorDroitPAMI2;
  KpmotorGauche = KpmotorGauchePAMI2;
  KimotorGauche = KimotorGauchePAMI2;
  PIDMotorDroit.SetTunings(KpmotorDroit,KimotorDroit,0);
  PIDMotorGauche.SetTunings(KpmotorGauche,KimotorGauche,0);
}
void updateKpKiPami3(){
  // fonction pour changer les Kp et Ki du PID du PAMI 3
  KpmotorDroit = KpmotorDroitPAMI3;
  KimotorDroit = KimotorDroitPAMI3;
  KpmotorGauche = KpmotorGauchePAMI3;
  KimotorGauche = KimotorGauchePAMI3;
  PIDMotorDroit.SetTunings(KpmotorDroit,KimotorDroit,0);
  PIDMotorGauche.SetTunings(KpmotorGauche,KimotorGauche,0);
}
void updateKpKiStart(){
  // fonction pour changer les Kp et Ki du PID du Start
  KpmotorDroit = KpmotorDroitStart;
  KimotorDroit = KimotorDroitStart;
  KpmotorGauche = KpmotorGaucheStart;
  KimotorGauche = KimotorGaucheStart;
  PIDMotorDroit.SetTunings(KpmotorDroit,KimotorDroit,0);
  PIDMotorGauche.SetTunings(KpmotorGauche,KimotorGauche,0);
}
void updateKpKi(){
  // fonction pour changer les Kp et Ki du PID
  KpmotorDroit = KpmotorDroitOriginal;
  KimotorDroit = KimotorDroitOriginal;
  KpmotorGauche = KpmotorGaucheOriginal;
  KimotorGauche = KimotorGaucheOriginal;
  PIDMotorDroit.SetTunings(KpmotorDroit,KimotorDroit,0);
  PIDMotorGauche.SetTunings(KpmotorGauche,KimotorGauche,0);
}

void motorDroit(MotorControl_t motorControl )
{
  // fonction pour controler le moteur droit en bas niveau

  /*Serial.print("reel:");
  Serial.println(OutputMotorDroit);
  */
  char d = motorControl.brakeOrAccDroite;
  SetpointMotorDroit = motorControl.speedDroit;
  InputMotorDroit = RPMDroit;
  PIDMotorDroit.Compute();
  if (OutputMotorDroit > maxSpeedPWM || isinf(OutputMotorDroit)){
    OutputMotorDroit = maxSpeedPWM;
  }
  /*if (OutputMotorDroit < minSpeedPWM && InputMotorDroit - SetpointMotorDroit < 0){
    OutputMotorDroit = 0;
    d = 'B';
  }*/
  //Serial.println(incSpeedDroit);
  //incSpeedDroit = speed;
  if(d =='F'){// forward
    digitalWrite(DroitA,LOW);
    analogWrite(DroitB, OutputMotorDroit);
    //digitalWrite(A1B,HIGH); 
  }else if (d =='R'){//rear
    analogWrite(DroitA,OutputMotorDroit);
    digitalWrite(DroitB,LOW);   
  }else if (d == 'B'){// brake via la back emf
    digitalWrite(DroitA,HIGH);
    digitalWrite(DroitB,HIGH);
  }else{// Turn motor OFF
    digitalWrite(DroitA,LOW);
    digitalWrite(DroitB,LOW);    
  }
}// motorA end

void motorGauche(MotorControl_t motorControl)
{
  // fonction pour controler le moteur gauche en bas niveau

  char d = motorControl.brakeOrAccGauche;
  SetpointMotorGauche = motorControl.speedGauche;
  InputMotorGauche = RPMGauche;
  PIDMotorGauche.Compute();

  /*Serial.print(" reel speed Gauche :");
  Serial.print(RPMGauche);
  Serial.print(" speed Gauche :");
  Serial.print(motorControl.speedGauche);*/
  if (OutputMotorGauche > maxSpeedPWM || isinf(OutputMotorGauche)){
    OutputMotorGauche = maxSpeedPWM;
  }
  /*if (OutputMotorGauche < minSpeedPWM && InputMotorGauche - SetpointMotorGauche < 0){
    OutputMotorGauche = 0;
    d = 'B';
  }*/
  if(d =='F'){// forward
    digitalWrite(GaucheA,LOW);
    analogWrite(GaucheB, OutputMotorGauche);
    //digitalWrite(A1B,HIGH); 
  }else if (d =='R'){//rear
    analogWrite(GaucheA,OutputMotorGauche);
    digitalWrite(GaucheB,LOW);   
  }else if (d == 'B'){// brake
    digitalWrite(GaucheA,HIGH);
    digitalWrite(GaucheB,HIGH);
  }else{// Turn motor OFF
    digitalWrite(GaucheA,LOW);
    digitalWrite(GaucheB,LOW);    
  }
}// motorA end

void updateEncoder(){
  //fonction pour mettre a jour les encodeurs et au besoins calculers les position XY, 
  // et lancer les controlleurs bas niveau si un certain nombre de tics est bien passé
  pasEncodeurGauche = encodeurGauche.read();
  encodeurGauche.write(0);
  pasEncodeurDroit = encodeurDroit.read();
  encodeurDroit.write(0);
  pasEncodeurGauchePIDMotor += pasEncodeurGauche;
  pasEncodeurDroitPIDMotor += pasEncodeurDroit;
  if (pasEncodeurGauche != 0 || pasEncodeurDroit != 0){
    calculXY();
  }
  float actuelTime = millis();//miliseconde
  if (abs(pasEncodeurDroitPIDMotor) >= seuilPIDMotor){
    RPMDroit = abs((pasEncodeurDroitPIDMotor/(8*4)) * (60*1000/(actuelTime - lastTimeDroit)));//*8 pour vitesse d'un tour, 1/... pour tour par seconde, *60 pour RPM
    lastTimeDroit = actuelTime;
    pasEncodeurDroitPIDMotor = 0;
  } else if (actuelTime - lastTimeDroit > 100) { // moteur a l arret
    RPMDroit = 0;
    lastTimeDroit = actuelTime;
    pasEncodeurDroitPIDMotor = 0;
  }
  if (abs(pasEncodeurGauchePIDMotor) >= seuilPIDMotor){
    RPMGauche = abs((pasEncodeurGauchePIDMotor/(8*4)) *  (60*1000/(actuelTime - lastTimeGauche)));//% du tour / temps en secode, *60 pour RPM
    lastTimeGauche = actuelTime;
    pasEncodeurGauchePIDMotor = 0;
  } else if (actuelTime - lastTimeGauche > 100){ // moteur a l arret
    RPMGauche = 0;
    lastTimeGauche = actuelTime;
    pasEncodeurGauchePIDMotor = 0;
  }
  motorDroit(motorControl);
  motorGauche(motorControl);
  
}
/*
// Fonction de rappel appelée lorsqu'un changement d'état est détecté sur l'une des broches de l'encodeur 1
void EncoderGaucheChange() {//---------------------------------------------------------------------------------------------------------------
  // Lire les états actuels des broches de l'encodeur 1
  //int etatActuelEncodeurGaucheA = digitalRead(PIN_ENCODEUR_GAUCHE_A);// d'office high car rising edge
  int etatActuelEncodeurGaucheB = digitalRead(PIN_ENCODEUR_GAUCHE_B);
  // Calculer le mouvement de l'encodeur 1 
  
  if (etatActuelEncodeurGaucheB == LOW ){
    if (mouvement1 == 1) {calculXYneed = true;}// calcul pos car changement de sens d'une roue
    mouvement1 = -1; // recule
  } else if (etatActuelEncodeurGaucheB == HIGH){
    if (mouvement1 == -1) {calculXYneed = true;}
    mouvement1 = 1; // avance
  } else {
    mouvement1 = 0;
  }
  pasEncodeurGauche += mouvement1;
  pasEncodeurGauchePIDMotor+= mouvement1;
  //Serial.println(pasEncodeurGauche);
  // Mettre à jour les états précédents des broches de l'encodeur 1
  if (abs(pasEncodeurGauche) >= seuilCalculDistance  || calculXYneed == true){
    calculXYneed = false;
    calculXY();
  } else if (abs(pasEncodeurGauchePIDMotor) >= seuilPIDMotor){
    float actuelTime = millis();
    RPMGauche = 1000*pasEncodeurGauchePIDMotor/((actuelTime - lastTimeGauche)*8*60);//*8 pour vitesse d'un tour, 1/... pour tour par seconde, *60 pour RPM
    lastTimeGauche = actuelTime;
    pasEncodeurGauchePIDMotor = 0;
    motorGauche(motorControl);
  }
 
}

// Fonction de rappel appelée lorsqu'un changement d'état est détecté sur l'une des broches de l'encodeur 2
void EncoderDroitChange() {//---------------------------------------------------------------------------------------------------------------
  // Lire les états actuels des broches de l'encodeur 2
  //int etatActuelEncodeurDroitA = digitalRead(PIN_ENCODEUR_2_A);// d'office high car rising edge
  int etatActuelEncodeurDroitB = digitalRead(PIN_ENCODEUR_DROIT_B);
  
  // Calculer le mouvement de l'encodeur 2
  if (etatActuelEncodeurDroitB == LOW ){
    if (mouvement2 == 1) {calculXYneed = true;}
    mouvement2 = -1; // recule 
  } else if (etatActuelEncodeurDroitB == HIGH ){
    if (mouvement2 == -1) {calculXYneed = true;}
    mouvement2 = 1; // avance
  } else {
    mouvement2 = 0;
  }
  pasEncodeurDroit += mouvement2;
  pasEncodeurDroitPIDMotor += mouvement2;
  //Serial.println(pasEncodeurDroit);
  if (abs(pasEncodeurDroit) >= seuilCalculDistance || calculXYneed == true ){
    calculXYneed = false;
    calculXY();
  } else if (abs(pasEncodeurDroitPIDMotor) >= seuilPIDMotor){
    float actuelTime = millis() ;//miliseconde -> seconde
    RPMDroit = 1000 * pasEncodeurDroitPIDMotor /((actuelTime - lastTimeDroit) *8*60);//*8 pour vitesse d'un tour, 1/... pour tour par seconde, *60 pour RPM
    lastTimeDroit = actuelTime;
    pasEncodeurDroitPIDMotor = 0;
    motorDroit(motorControl);
  }
}*/


void calculXY(){//---------------------------------------------------------------------------------------------------------------
  // Fonction pour calculer la position du robot en fonction des distances parcourues par les
  // roues et capté par les encodeurs

  // encodeur de 8 puls par tour soit 960 par tour de roue
  // Convertir les valeurs des encodeurs en tours de roues
  float toursGauche = (float)pasEncodeurGauche / PULS_TOUR_ROUE;
  float toursDroite = (float)pasEncodeurDroit / PULS_TOUR_ROUE;
  
  // Calculer la distance parcourue par chaque roue
  float distanceParcourueGauche = toursGauche * 2 * RAYON_ROUE * PI;
  float distanceParcourueDroite = toursDroite * 2 * RAYON_ROUE * PI;
  
  // Calculer la distance parcourue par le robot
  float distanceParcourue = (distanceParcourueGauche + distanceParcourueDroite) / 2.0;

  // Calculer l'angle de rotation du robot
  float deltaAngle = (distanceParcourueDroite - distanceParcourueGauche) / DISTANCE_ENTRE_ROUES;
  //angleRotation += angleRotationPrecedent;

  // Mettre à jour la position du robot
  positionRobot.positionX += distanceParcourue * cos(positionRobot.angleRotation + deltaAngle/2);
  positionRobot.positionY += distanceParcourue * sin(positionRobot.angleRotation + deltaAngle/2);

  positionRobot.angleRotation += deltaAngle;
}


