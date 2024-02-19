// Pins
// IN1
const uint8_t C_L = 14, C_R = 3;
// IN2
const uint8_t D_L = 15, D_R = 4;
// Enable
const uint8_t PWM_L = 22, PWM_R = 23;
// Current sensors
// const uint8_t CURRENT_L = 41, CURRENT_R = 40;

double max_curl = 0, max_curr = 0;

// Create encoder objects with the pins A and B

Encoder enc_l(25, 26);

// Level-shifter pin
const int LEVEL_SHIFTER = 2;

int sens_time, current_time;
int printed = 0;


void setup() {

  // Create a Serial interface
  Serial.begin(115200);

  // Enable the level shifter
  pinMode(LEVEL_SHIFTER, OUTPUT);
  digitalWrite(LEVEL_SHIFTER, HIGH);

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
  analogWrite(PWM_L, 0);
  analogWrite(PWM_R, 0);

  enc_l.write(0);

  sens_time = 0;
}

void loop() {
  // Get time
  current_time = millis();
  
  if (current_time - sens_time > 20) {
    sens_time = current_time;

    if (current_time > 6000){
        analogWrite(PWM_L, 0);
        analogWrite(PWM_R, 0);
    } else if (current_time > 1000) {
        sens_time = current_time;
        analogWrite(PWM_L, 120);
        analogWrite(PWM_R, 120);
        Serial.printf("%d\t%d\n", current_time, (int) enc_l.read());
    }
  }

}