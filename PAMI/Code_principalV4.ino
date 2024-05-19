#include "Encodeur_Moteur.h"
#include "Global_Variable.h"


//TARGET
double SetpointTARGET, InputTARGET, OutputTARGET;
PID PIDTARGET(&InputTARGET, &OutputTARGET, &SetpointTARGET, KpEncodeur, KIEncodeur, KdEncodeur, DIRECT);

//declaration Line Follower
const int capteur1Pin = A0;// gauche
const int capteur2Pin = A1;
const int capteur3Pin = A2;// centre
const int capteur4Pin = A3;
const int capteur5Pin = A4;// droit
int line[5];
int errorTab[3];
int numberLine = 0;
int sensiLine = 800;
float error = 0;


//declaration SERVO
Servo myservo;
const int angleRobotAdvers = 55;
const int angleBas =90+20;
const int anglePlante = 90;
const int angleJardiniere = 60;

//declaration SONAR
const byte TRIG_PIN = 9;
const byte ECHO_PIN = 6;
UltraSonicDistanceSensor distanceSensor(TRIG_PIN, ECHO_PIN); //initialisation class HCSR04 (trig pin , echo pin)
bool start = true;
bool plante = false;
int timerPlante =0;

//declaration starting corde
const int pinStartingCorde = A5;
const int ThresholdStartingCorde = 600;
long tempsDebut;
long tempsActuel;
long startTime = 90L * 1000;//90L * 1000; // time en miliseconde
long endTime = startTime + 9.5L *1000; //L pour signifier que cest un long
long delayPAMI23 = 1.5L *1000;
long startTimeSonarRex = startTime + 0.8L *1000;
long startTimeSonarPAMI23 = startTimeSonarRex + delayPAMI23;

//declaration entree en zone
bool zone = false;
int previousLine[5];
int zoneCount = 0;
bool fini = false;
bool verbose = true; // laisser en TRUE!! car modofie le temps pour les Ki...
bool SonarPami2 = false;

void setup() {
  if (verbose){Serial.begin(9600);}
  if (PAMI == 1){
    positionRobot  = {AngleInitiale,PosInitialeX_PAMI1,PosInitialeY_PAMI1};
    KpEncodeur = 140;//100
    KdEncodeur = 2;
    KIEncodeur = 5;
  } else if (PAMI == 2){
    positionRobot  = {AngleInitiale,PosInitialeX_PAMI2,PosInitialeY_PAMI2};
    KpEncodeur = 140;//840;//100
    KdEncodeur = 0;
    KIEncodeur = 100;//300;
    maxSpeed = 1100;
    normalSpeed = 800;
    maxSpeedPWM = 255;
  } else if (PAMI == 3){
    positionRobot  = {AngleInitiale,PosInitialeX_PAMI3,PosInitialeY_PAMI3};
    KpEncodeur = 140;//100
    KdEncodeur = 2;
    KIEncodeur = 20;
  }
  PIDTARGET.SetTunings(KpEncodeur,KIEncodeur,KdEncodeur);
  initialisePID();
  PIDTARGET.SetMode(AUTOMATIC);
  PIDTARGET.SetOutputLimits(-maxSpeed,maxSpeed);
  pinMode(13, OUTPUT);//led demarrage
  pinMode(DroitA,OUTPUT);// define pin as output
  pinMode(DroitB,OUTPUT);
  pinMode(GaucheA,OUTPUT);
  pinMode(GaucheB,OUTPUT);  
  myservo.attach(10);  // Attache le servo au pin 10
  //pinMode(TRIG_PIN, OUTPUT);// pin pour sonar
  //pinMode(ECHO_PIN, INPUT);

  //declaration boutton couleur BLUE JAUNE
  ezButton toggleSwitch(0);  // create ezButton object that attach to pin 7;
  toggleSwitch.setDebounceTime(50); // temps de stabilite avant que le bouton soit considere comme appuyé  
  if (toggleSwitch.getState() == HIGH){
    COLOR = "YELLOW";
  } else {
    COLOR = "BLUE";
    //inversion PAMI1
    TARGET_X_PAMI1 =  TARGET_X_PAMI1_BLEU;
    TARGET_Y_PAMI1 = -TARGET_Y_PAMI1;
    LIMITE_Y_ZONE_PAMI1 = -LIMITE_Y_ZONE_PAMI1;
    LIMITE_Y_STOP_ZONE_PAMI1 = -LIMITE_Y_STOP_ZONE_PAMI1;

    //inversion PAMI2
    TARGET_Y_PAMI2 = -TARGET_Y_PAMI2;
    TARGET_YPRIME_PAMI2 =  -TARGET_YPRIME_BLUE_PAMI2;
    TARGET_Y_IN_ZONE_PAMI2 = - TARGET_Y_IN_ZONE_BLUE_PAMI2;
    //LIMITE_Y_ZONE_PAMI2 = -LIMITE_Y_ZONE_PAMI2;
    LIMITE_Y_STOP_ZONE_PAMI2 = -LIMITE_Y_STOP_ZONE_PAMI2;
    

    //inversion PAMI3 
    TARGET_Y_PAMI3 = -TARGET_Y_PAMI3;
    TARGET_YPRIME_PAMI3 = -TARGET_YPRIME_PAMI3;
    LIMITE_Y_ZONE_PAMI3 = -LIMITE_Y_ZONE_PAMI3;
    LIMITE_Y_STOP_ZONE_PAMI3 = -LIMITE_Y_STOP_ZONE_PAMI3;

    positionRobot.positionY = - positionRobot.positionY;
  }
  myservo.write(0);             
  delay (500);                    
  myservo.write(angleBas);
  delay(500);
  if (COLOR == "BLUE"){
    myservo.write(0);             
    delay (500);                    
    myservo.write(angleBas);
    delay(500);
  }
  //------------------------------------------------------STARTING CORDE ----------------------------------------
  while (analogRead(pinStartingCorde) <= ThresholdStartingCorde ){ 
    digitalWrite(13, HIGH);
    delay(90);
    digitalWrite(13, LOW);
    delay(90);
  }
  digitalWrite(13, LOW);
  tempsDebut = millis();
  tempsActuel = millis();  // Récupère le temps actuel
  //Serial.println(tempsActuel - tempsDebut);
  //------------------------------------------------------delai demarrage 90s -------------------------------------
  while ( tempsActuel - tempsDebut < startTime ){  
    if (analogRead(pinStartingCorde) <= ThresholdStartingCorde){// si cordon rebranché, reset le temps
      tempsDebut = millis();
    }
    if (verbose){
      Serial.print(tempsActuel - tempsDebut);
      Serial.print("startTime :");
      Serial.println(startTime);
    }
    delay(100);
    tempsActuel = millis();
    lastTimeGauche = millis();
    lastTimeDroit = millis();
  }  
  //------------------------------------------------------delai supplementaire PAMI 2 et 3 ------------------------
  if (PAMI == 2 || PAMI == 3){
    while ( tempsActuel - tempsDebut - startTime < delayPAMI23 ){  
      delay(10);
      tempsActuel = millis();
    }
  }  
  // -------------------------------------------------------Reset Integrateur ET Encodeur -------------------------------
  //reset de tout les integrateurs
  PIDTARGET.SetMode(MANUAL);//reset de l'integrateur encodeurs
  OutputTARGET = 0;
  PIDTARGET.SetMode(AUTOMATIC);
  resetIntegrateurPIDMotor();//reset des moteurs ET ENCODEURS
  if (PAMI == 1){// et reset des pos initiale si qql a bouger sans faire espret
    positionRobot  = {AngleInitiale,PosInitialeX_PAMI1,PosInitialeY_PAMI1};
  } else if (PAMI == 2){
    positionRobot  = {AngleInitiale,PosInitialeX_PAMI2,PosInitialeY_PAMI2};
  } else if (PAMI == 3){
    positionRobot  = {AngleInitiale,PosInitialeX_PAMI3,PosInitialeY_PAMI3};
  }
  if (COLOR == "BLUE"){
    positionRobot.positionY = - positionRobot.positionY;
  }

  bool change_point_pami = false;
  myservo.detach();
  if (PAMI == 3) {
    updateKpKiStart();
  } else if (PAMI == 2){
    updateKpKiPami2();
  }


  // ----------------------------------------------------------------boucle PRINCIPAL Jusqua ZONE--------------------------------------------------------------------------
  while (zone == false){  
    if (PAMI == 3 && millis() - tempsDebut>= 800 ){
      updateKpKi();
    }
    updateEncoder();
    finish();
    if (verbose){printPositionXY();}
    if (PAMI == 1) {//------------------------------------- PAMI1
      //fini = positionRobot.positionX <= TARGET_X_PAMI1 && abs(positionRobot.positionY) >= abs(TARGET_Y_PAMI1) - 50 ;
      zone= (abs(positionRobot.positionY )>= abs(LIMITE_Y_ZONE_PAMI1));
      if (zone){
        myservo.attach(10);
        myservo.write(0);
      }
    } else if (PAMI ==2){//-------------------------------- PAMI2
      //fini = (abs(positionRobot.positionY )>= abs(LIMITE_Y_LIGNE_PAMI2)) && (abs(positionRobot.positionX )>= abs(LIMITE_X_LIGNE_PAMI2));
      zone = (abs(positionRobot.positionX )>= abs(LIMITE_X_ZONE_PAMI2));

      float distance_with_target =sqrt((TARGET_X_PAMI2-positionRobot.positionX)*(TARGET_X_PAMI2-positionRobot.positionX) + (TARGET_Y_PAMI2-positionRobot.positionY)*(TARGET_Y_PAMI2-positionRobot.positionY));
      if (distance_with_target < 200 && change_point_pami == false){
        change_point_pami = true;          
        TARGET_X_PAMI2 = TARGET_XPRIME_PAMI2;
        TARGET_Y_PAMI2 = TARGET_YPRIME_PAMI2;
        PIDTARGET.SetMode(MANUAL);//reset de l'integrateur
        OutputTARGET = 0;
        PIDTARGET.SetMode(AUTOMATIC);

      }
    } else if (PAMI == 3) {//-------------------------------- PAMI3
      //float distance_with_target =sqrt((TARGET_X_PAMI3-positionRobot.positionX)*(TARGET_X_PAMI3-positionRobot.positionX) + (TARGET_Y_PAMI3-positionRobot.positionY)*(TARGET_Y_PAMI3-positionRobot.positionY));
      if (abs(positionRobot.positionY) >= abs(LIMITE_Y_ZONE_PAMI3) && abs(positionRobot.positionX) >= LIMITE_X_ZONE_PAMI3 && change_point_pami == false){
        zone = true;
        change_point_pami = true;          
        TARGET_X_PAMI3 = TARGET_XPRIME_PAMI3;
        TARGET_Y_PAMI3 = TARGET_YPRIME_PAMI3;
        myservo.attach(10);
        myservo.write(0);
      }
    }
    detecteurObstaclePASSIF(start);
    if (start == true){// start de detecteur obstacle
      towardTarget();
      //Serial.println(PIDTARGET.GetKp());
      /*int speed = (int)(maxSpeed * (cos(millis() / 1000.0) + 1)/2);
      motorControl.brakeOrAccDroite = 'F';
      motorControl.brakeOrAccGauche = 'F';
      motorControl.speedDroit =speed;
      motorControl.speedGauche = speed;
      motorDroit(motorControl);
      motorGauche(motorControl);*/
    }
  }
  //-----------------------------------------------------------------CONTROL EN ANGLE dans ZONE---------------------------------------------------------------------------
  motorControl.brakeOrAccDroite = 'B';
  motorControl.brakeOrAccGauche = 'B';
  motorControl.speedDroit =0;
  motorControl.speedGauche = 0;
  motorDroit(motorControl);
  motorGauche(motorControl);
  if (PAMI == 1){
    if (COLOR == "BLUE"){
      TARGET_X_PAMI1 = TARGET_XPRIME_PAMI1_BLEU;
    }
    myservo.attach(10);
    myservo.write(0);
    delay(200);
    SetpointTARGET = atan2((TARGET_Y_PAMI1-positionRobot.positionY),(TARGET_X_PAMI1-positionRobot.positionX));
    double errorPositionControl = SetpointTARGET - positionRobot.angleRotation;
    while (abs(errorPositionControl) > 0.05 ){ // tant que erreur change pas de signe
      updateEncoder();
      finish();
      SetpointTARGET = atan2((TARGET_Y_PAMI1-positionRobot.positionY),(TARGET_X_PAMI1-positionRobot.positionX));
      errorPositionControl = SetpointTARGET - positionRobot.angleRotation; 
      detecteurObstaclePASSIF(start);
      if (start == true){
        if (sign(errorPositionControl) >0){
          motorControl.brakeOrAccDroite = 'F';
          motorControl.brakeOrAccGauche = 'B';
          motorControl.speedDroit = normalSpeedInZone/3;
          motorControl.speedGauche = 0;
          motorDroit(motorControl);
          motorGauche(motorControl);
        } else{
          motorControl.brakeOrAccDroite = 'B';
          motorControl.brakeOrAccGauche = 'F';
          motorControl.speedDroit = 0;
          motorControl.speedGauche = normalSpeedInZone/3;
          motorDroit(motorControl);
          motorGauche(motorControl);
        }
      }
    }
    motorControl.brakeOrAccDroite = 'B';
    motorControl.brakeOrAccGauche = 'B';
    motorControl.speedDroit =0;
    motorControl.speedGauche = 0;
    motorDroit(motorControl);
    motorGauche(motorControl);
    delay(200);
  } else if (PAMI == 2){
    TARGET_X_PAMI2 = TARGET_X_IN_ZONE_PAMI2 ;
    TARGET_Y_PAMI2 = TARGET_Y_IN_ZONE_PAMI2 ;
    SonarPami2 = true;
    myservo.attach(10);
    myservo.write(angleRobotAdvers);
    delay(200);
    SetpointTARGET = atan2((TARGET_Y_PAMI2-positionRobot.positionY),(TARGET_X_PAMI2-positionRobot.positionX));
    double errorPositionControl = SetpointTARGET - positionRobot.angleRotation;
    while (abs(errorPositionControl) > 0.05 ){ // tant que erreur change pas de signe
      updateEncoder();
      finish();
      SetpointTARGET = atan2((TARGET_Y_PAMI2-positionRobot.positionY),(TARGET_X_PAMI2-positionRobot.positionX));
      errorPositionControl = SetpointTARGET - positionRobot.angleRotation; 
      detecteurObstaclePASSIF(start);
      if (start == true){
        if (sign(errorPositionControl) >0){
          motorControl.brakeOrAccDroite = 'F';
          motorControl.brakeOrAccGauche = 'B';
          motorControl.speedDroit = normalSpeedInZone/3;
          motorControl.speedGauche = 0;
          motorDroit(motorControl);
          motorGauche(motorControl);
        } else{
          motorControl.brakeOrAccDroite = 'B';
          motorControl.brakeOrAccGauche = 'F';
          motorControl.speedDroit = 0;
          motorControl.speedGauche = normalSpeedInZone/3;
          motorDroit(motorControl);
          motorGauche(motorControl);
        }
      }
    }
    motorControl.brakeOrAccDroite = 'B';
    motorControl.brakeOrAccGauche = 'B';
    motorControl.speedDroit =0;
    motorControl.speedGauche = 0;
    motorDroit(motorControl);
    motorGauche(motorControl);
    delay(200);
  } else if (PAMI==3){
    myservo.attach(10);
    myservo.write(0);
    delay(200);
    SetpointTARGET = atan2((TARGET_Y_PAMI3-positionRobot.positionY),(TARGET_X_PAMI3-positionRobot.positionX));
    double errorPositionControl = SetpointTARGET - positionRobot.angleRotation;
    while (abs(errorPositionControl) > 0.05 ){ // tant que erreur change pas de signe
      updateEncoder();
      finish();
      SetpointTARGET = atan2((TARGET_Y_PAMI3-positionRobot.positionY),(TARGET_X_PAMI3-positionRobot.positionX));
      errorPositionControl = SetpointTARGET - positionRobot.angleRotation; 
      detecteurObstaclePASSIF(start);
      if (start == true){
        if (sign(errorPositionControl) >0){
          motorControl.brakeOrAccDroite = 'F';
          motorControl.brakeOrAccGauche = 'B';
          motorControl.speedDroit = normalSpeedInZone/3;
          motorControl.speedGauche = 0;
          motorDroit(motorControl);
          motorGauche(motorControl);
        } else{
          motorControl.brakeOrAccDroite = 'B';
          motorControl.brakeOrAccGauche = 'F';
          motorControl.speedDroit = 0;
          motorControl.speedGauche = normalSpeedInZone/3;
          motorDroit(motorControl);
          motorGauche(motorControl);
        }
      }
    }
    motorControl.brakeOrAccDroite = 'B';
    motorControl.brakeOrAccGauche = 'B';
    motorControl.speedDroit =0;
    motorControl.speedGauche = 0;
    motorDroit(motorControl);
    motorGauche(motorControl);
    delay(200);
  }


  //----------------------------------------------------------------------------BOUCLE EN ZONE-------------------------------------------------------------------
  //arrive en zone, reduit vitesse
  normalSpeed = normalSpeedInZone;
  PIDTARGET.SetOutputLimits(-maxSpeedInZone,maxSpeedInZone);
  KpEncodeur = KpEncodeur;
  KIEncodeur = KIEncodeur / 3;
  while (fini == false){
    updateEncoder();
    finish();
    if (PAMI == 1) {//------------------------------------- PAMI1
      fini = abs(positionRobot.positionY) >= abs(LIMITE_Y_STOP_ZONE_PAMI1) ;
    } else if (PAMI ==2){//-------------------------------- PAMI2
      fini = (abs(positionRobot.positionY )>= abs(LIMITE_Y_STOP_ZONE_PAMI2)) && (abs(positionRobot.positionX )>= abs(LIMITE_X_STOP_ZONE_PAMI2));
    } else if (PAMI == 3){//-------------------------------- PAMI3
      fini = positionRobot.positionX <= LIMITE_X_STOP_ZONE_PAMI3;
    }     
    detecteurObstaclePASSIF(start);         
    if (start == true){// start de detecteur obstacle
      towardTarget();
      if (verbose){printPositionXY();}
    }
  }
  motorControl.brakeOrAccDroite = 'B';
  motorControl.brakeOrAccGauche = 'B';
  motorControl.speedDroit =0;
  motorControl.speedGauche = 0;
  motorDroit(motorControl);
  motorGauche(motorControl);
  myservo.attach(10); 
}



void loop() { //----------------------------------------------------FINISH, mets bien le nez-------------------------------------------------------------------------------------
  if (PAMI == 3 || PAMI == 1|| PAMI == 2){
    myservo.attach(10); 
    myservo.write(angleJardiniere);
  } else {
    myservo.write(0);             
    delay (500);  
    myservo.write(90);
  }
  motorControl.brakeOrAccDroite = 'B';
  motorControl.brakeOrAccGauche = 'B';
  motorControl.speedDroit =0;
  motorControl.speedGauche = 0;
  motorDroit(motorControl);
  motorGauche(motorControl);
  delay(500);
} //-----------------------------------------------------------------------------------------------------------------------------------------



void finish(){
  // fonction de fin de match
  tempsActuel = millis();
  if (tempsActuel -tempsDebut >= endTime){
    if (PAMI == 3 || PAMI == 1|| PAMI == 2){
      myservo.attach(10); 
      myservo.write(angleJardiniere);
    } else {
      myservo.write(0);             
      delay (500);  
      myservo.write(90);
    }
    while (true){
      motorControl.brakeOrAccDroite = 'B';
      motorControl.brakeOrAccGauche = 'B';
      motorControl.speedDroit =0;
      motorControl.speedGauche = 0;
      motorDroit(motorControl);
      motorGauche(motorControl);
      delay(500);
    }
  }
}


void towardTarget(){
  // fonction pour aller vers la cible en fonction de la position actuelle
  // I   |  III
  //-----x-----
  // II  |  IV
  float targetY=0;
  float targetX=0;
  if (PAMI == 1){
    targetY = TARGET_Y_PAMI1;
    targetX = TARGET_X_PAMI1;
  } else if (PAMI == 2){
    targetY = TARGET_Y_PAMI2;
    targetX = TARGET_X_PAMI2;
  } else if (PAMI == 3){
    targetY = TARGET_Y_PAMI3;
    targetX = TARGET_X_PAMI3;
  }
  double lastSetpointTarget = SetpointTARGET;
  SetpointTARGET = atan2((targetY-positionRobot.positionY),(targetX-positionRobot.positionX));
  if (abs(SetpointTARGET-lastSetpointTarget) >=6){ // si saute un tour et passe de -pi a +pi(ou inversement)
    SetpointTARGET +=  2 * PI * sign(lastSetpointTarget);
  }
  InputTARGET = positionRobot.angleRotation;
  PIDTARGET.Compute();
  motorControl.speedDroit = normalSpeed + OutputTARGET;
  // Serial.println(OutputTARGET);
  motorControl.speedGauche = normalSpeed - OutputTARGET;// qd erreur max(20), vitesse de 0
  motorControl.brakeOrAccDroite = 'F';
  motorControl.brakeOrAccGauche = 'F';
  if (motorControl.speedDroit > maxSpeed){
    motorControl.speedDroit = maxSpeed;
  } else if (motorControl.speedDroit < 0){
    motorControl.speedDroit = 0;
    motorControl.brakeOrAccDroite = 'B';
  }
  if (motorControl.speedGauche > maxSpeed){
    motorControl.speedGauche = maxSpeed;
  }else if (motorControl.speedGauche < 0){
    motorControl.speedGauche = 0;
    motorControl.brakeOrAccGauche = 'B';
  }

  motorDroit(motorControl);
  motorGauche(motorControl);
}
 

void printPositionXY(){
  // fonction pour afficher la position du robot, ainsi que la cible
  // Affichage de la position des encodeurs
  Serial.print(" X: ");
  Serial.print(positionRobot.positionX);
  Serial.print(" Y: ");
  Serial.print(positionRobot.positionY);
  Serial.print(" angle de rot: ");
  Serial.print(positionRobot.angleRotation,6);
  float targetY=0;
  float targetX=0;
  if (PAMI == 1){
    targetY = TARGET_Y_PAMI1;
    targetX = TARGET_X_PAMI1;
  } else if (PAMI == 2){
    targetY = TARGET_Y_PAMI2;
    targetX = TARGET_X_PAMI2;
  } else if (PAMI == 3){
    targetY = TARGET_Y_PAMI3;
    targetX = TARGET_X_PAMI3;
  }
  Serial.print(" target (x,y): ");
  Serial.print(targetX);
  Serial.print(targetY);
  Serial.print(" angleTarget :");
  Serial.print(SetpointTARGET);
  Serial.print(" error Angle :");
  Serial.println( SetpointTARGET - positionRobot.angleRotation);
}

void suiviLigne(float error){
  // fonction pour suivre la ligne en fonction de l'erreur
  // si error <0, ligne a GAUCHE
  // si error >0, ligne a DROITE
  motorControl.brakeOrAccDroite = 'F';
  motorControl.brakeOrAccGauche = 'F';
  motorControl.derivatieError = error - errorPrevious;
  motorControl.IntErrorDroit = motorControl.IntErrorDroit + error ;
  motorControl.IntErrorGauche = motorControl.IntErrorGauche + error;
  motorControl.speedDroit = normalSpeed - error*Kp  - motorControl.IntErrorDroit * KI - motorControl.derivatieError * Kd;
  motorControl.speedGauche = normalSpeed + error*Kp  + motorControl.IntErrorGauche * KI + motorControl.derivatieError * Kd;
  if (motorControl.speedDroit > maxSpeed){
    motorControl.speedDroit = maxSpeed;
    motorControl.IntErrorDroit = 0;
  } else if (motorControl.speedDroit < 0){
    motorControl.speedDroit = 0;
    motorControl.IntErrorDroit = 0;
  }
  if (motorControl.speedGauche > maxSpeed){
    motorControl.speedGauche = maxSpeed;
    motorControl.IntErrorDroit = 0;
  }else if (motorControl.speedGauche < 0){
    motorControl.speedGauche = 0;
    motorControl.IntErrorGauche = 0;
  }
  motorDroit(motorControl);
  motorGauche(motorControl);

}


void print_Line(int line[]){
  // fonction pour afficher la ligne detectee
  //Serial.print(lineSTR);
  Serial.print(line[0]);
  Serial.print(line[1]);
  Serial.print(line[2]);
  Serial.print(line[3]);
  Serial.println(line[4]);
  delay(100);
}

void print_SONAR(){
  // fonction pour afficher la distance du sonar
  Serial.println(distanceSensor.measureDistanceCm());
}

void verifTime(int tempsDebut, int endTime){
  // fonction pour verifier le temps et arreter le robot au besoin
  tempsActuel = millis();  // Récupère le temps actuel
  if(tempsActuel - tempsDebut >= endTime){
    motorControl.brakeOrAccDroite = 'B'; // brake and stop all
    motorControl.brakeOrAccGauche = 'B';
    motorControl.speedDroit =0;
    motorControl.speedGauche = 0;
    motorDroit(motorControl);
    motorGauche(motorControl);
    //Serial.println("STOOOPPP");
    while ( true ){
      delay(1000);
    }
  }
}





void detecteurObstacleACTIF(bool& start, bool& plante, int& timerPlante){
  // ancienne fonction de detecteur d'obstacle, ne sert plus mais qui avait pour objectif de detecter de maniere actif
  // en jouant avec le servo pour detecter la hauteur de l'objet devant nous
  int angleServo = myservo.read();
  int distSonar = distanceSensor.measureDistanceCm();
  if (distSonar == -1){ distSonar = 200;} //distsonar = infini
  if (distSonar < 10){
    motorControl.brakeOrAccDroite = 'B'; // brake and stop all
    motorControl.brakeOrAccGauche = 'B';
    motorControl.speedDroit =0;
    motorControl.speedGauche = 0;
    motorDroit(motorControl);
    motorGauche(motorControl);
    myservo.write(anglePlante); // regarde au milieu
    delay(100);
    distSonar = distanceSensor.measureDistanceCm(); // puis reprend mesure
    if (distSonar >20){ // rien en au milieu donc cest un pami
      start = false; 
      plante = false;
      myservo.write(angleBas);
      delay(100);
    } else { // ca peut etre un robot adv ou une plante
      myservo.write(angleRobotAdvers); // regarde en haut
      delay(100);
      distSonar = distanceSensor.measureDistanceCm();
      if (distSonar > 20){
        plante = true; // plante donc go
        start = true;
      } else {
        plante = false;// robot adverse
        start = false;
      }
     
    }
  } else if( angleServo >= (angleRobotAdvers - 5) &&   angleServo <= (angleRobotAdvers + 5) ){ 
    // si servo en position robot adv avec range de -5 +5°
    if (distSonar<20){ // tjr robot adv present
      start = false; 
    } else { // robot parti
      start = true;
      if (plante == true){// si plante redescent pas tout de suite
        timerPlante +=1;
        if (timerPlante >= 200){ 
          myservo.write(angleBas); // redecent car timer passé
          timerPlante = 0;
        }
      } else {
        myservo.write(angleBas); // revient a pos initiale du sonar
      }
    }
  } else {
    start = true;
  }
  
}

void detecteurObstaclePASSIF(bool& start){
  // fonction de detecteur d'obstacle passif, qui ne fait que regarder la distance du sonar et arrete le robot si besoin
  //myservo.write(angleBas);
  tempsActuel = millis();
  if ((PAMI == 1 && (tempsActuel - tempsDebut> startTimeSonarRex)) || (PAMI !=1 && (tempsActuel - tempsDebut > startTimeSonarPAMI23) ) ){
    // myservo.attach(10);
    // myservo.write(0);
    // Serial.println("TIMER SONAR OK");
    int distSonar = distanceSensor.measureDistanceCm();
    if (distSonar == -1){ distSonar = 200;} //distsonar = infini
    int arretAdversaire;
    if (SonarPami2){ 
      arretAdversaire = 35;
    }else{
      arretAdversaire = 15;//25
    } 
    if (PAMI == 3){
      arretAdversaire = 15;
    }
    if (distSonar < arretAdversaire){
      motorControl.brakeOrAccDroite = 'B'; // brake and stop all
      motorControl.brakeOrAccGauche = 'B';
      motorControl.speedDroit =0;
      motorControl.speedGauche = 0;
      motorDroit(motorControl);
      motorGauche(motorControl);
      start = false;
    } else {
      motorControl.brakeOrAccDroite = 'F'; // brake and stop all
      motorControl.brakeOrAccGauche = 'F';
      start = true;
    }
  }
  
}


void lineFollower(int line[] , int errorTab[], int& numberLine){
  // fonction pour recuperer les donnée des suiveurs de lignes

  // line est un tableau avec les sensor qui detecte les lignes, 
  //errorTab un tableau avec les erreurs moyen par rapport au differente ligne en cm
  // numberLine est le nombre de ligne DISTINCTE detecte

  //int* line = new int[5];
  /*line[0] = analogRead(capteur1Pin);
  line[1] = analogRead(capteur2Pin);
  line[2] = analogRead(capteur3Pin);
  line[3] = analogRead(capteur4Pin);
  line[4] = analogRead(capteur5Pin);*/
  if (analogRead(capteur1Pin) >= sensiLine) {line[0] =1;} else { line[0]  = 0;}
  if (analogRead(capteur2Pin) >= sensiLine) {line[1] =1;} else { line[1]  = 0;}
  if (analogRead(capteur3Pin) >= sensiLine) {line[2] =1;} else { line[2]  = 0;}
  if (analogRead(capteur4Pin) >= sensiLine) {line[3] =1;} else { line[3]  = 0;}
  if (analogRead(capteur5Pin) >= sensiLine) {line[4] =1;} else { line[4]  = 0;}
  //print_Line(line);
  /*lineSTR = "";
  for (int i = 0; i<5;i++){
    lineSTR += String(line[i]);
  }
  */
  bool previous = false;
  numberLine = 0;
  int sommeForMeans = 0;
  int errorLine = 0; // -20 -10 0 10 20
  for (int i=0;i < 5; i++){ // detecte
    if (line[i] == 1){
      sommeForMeans = sommeForMeans +1;
      errorLine = errorLine + ( -20 + 10 * i) ;
      previous = true;
    } else if (line[i] == 0) { //detecte plus (line ==0)
      if (previous == true){
        errorLine = errorLine/sommeForMeans ; // -20 - 10 = -30/2 = -15
        errorTab[numberLine] = errorLine; // [errorLine1, errorLine2, errorLine3] max 3 ligne distincte detecte
        sommeForMeans = 0;
        errorLine =0;
        numberLine = numberLine+1;
      }
      previous = false;
    }
  }
  if (previous == true){ // regarde pour le dernier point
    errorLine = errorLine /sommeForMeans ; // -20 - 10 = -30/2 = -15
    errorTab[numberLine] = errorLine; // [errorLine1, errorLine2, errorLine3]
    sommeForMeans = 0;
    numberLine +=1;
  }
  
}

