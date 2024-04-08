// Pins
// IN1
const uint8_t C_L = 14, C_R = 3;
// IN2
const uint8_t D_L = 15, D_R = 4;
// Enable
const uint8_t PWM_L = 22, PWM_R = 23;
// Current sensors
const uint8_t CURRENT_L = 41, CURRENT_R = 40;
int sens_time, current_time;

double max_curl = 0, max_curr = 0;

//        LEFT     RIGHT
// FWD    CLDH     CHDL
// BWD    CHDL     CLDH

// PWM = 125 + Wheel blocked : Current = 230mA

void setup() {

  // Start serial communication
  Serial.begin(115200);


  // Output pins
  pinMode(C_L, OUTPUT);
  pinMode(D_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(C_R, OUTPUT);
  pinMode(D_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  // FWD
  
  digitalWrite(C_L, LOW);
  digitalWrite(D_L, HIGH);
  digitalWrite(C_R, HIGH);
  digitalWrite(D_R, LOW);

  // BWD 
  /*
  digitalWrite(C_L, HIGH);
  digitalWrite(D_L, LOW);
  digitalWrite(C_R, LOW);
  digitalWrite(D_R, HIGH);*/
  
  analogWriteFrequency(PWM_L, 20e3);
  analogWriteFrequency(PWM_R, 20e3);
  analogWrite(PWM_L, 75);
  analogWrite(PWM_R, 75);

  // Input pins
  pinMode(CURRENT_L, INPUT);
  pinMode(CURRENT_R, INPUT);

  sens_time = 0;
}

void loop() {
  // Get time
  current_time = millis();

  if (current_time - sens_time > 200){
    // print to the serial port the current
    max_curl = analogRead(CURRENT_L)/0.5*3.3/1023;
    max_curr = analogRead(CURRENT_R)/0.5*3.3/1023;
    Serial.printf("Left  %.3f\n", max_curl);
    Serial.printf("Right %.3f\n", max_curr);
    // Reset the time value for sensing
    sens_time = current_time;
  }

}