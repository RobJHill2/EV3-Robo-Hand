// Clock:
double last_time = 0;
double dt;

// Index Movement:
#define EN 13 // sets motor speed (PWM)
#define O1 22
#define O2 23 // direction controls

// Index Rotary Encoder:
#define YELLOW_SIG 2 // interrupt 
#define BLUE_SIG 53 // bool
volatile int degrees;

// Index Input:

#define INDEX_IN A1

int initPotVal;

// PID Controller:
#define Kp 1
#define Ki 0.9
#define Kd 0.05

#define tolerance 0

#define speedFactor 1.3

double integral = 0;
double prev_error = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  pinMode(EN, OUTPUT);
  pinMode(O1, OUTPUT);
  pinMode(O2, OUTPUT);

  pinMode(INDEX_IN, INPUT);

  pinMode(YELLOW_SIG, INPUT);
  pinMode(BLUE_SIG, INPUT);

  initPotVal = analogRead(INDEX_IN);

  degrees = 0;

  attachInterrupt(digitalPinToInterrupt(YELLOW_SIG), countDegrees, CHANGE);

  Serial.println("hello world"); // prints in serial manager 
}


void loop() {
  double now = millis();
  dt  = (now - last_time) / 1000.00;
  last_time = now;

  int potVal = analogRead(INDEX_IN);
  int setpoint = (potVal - initPotVal) / 5;
  setpoint = round((float)setpoint / 10)*10;
  // setpoint in degrees from starting position
  double pidOutput = pid(setpoint - degrees); // if degrees > setpoint: -ve error
  //Serial.println(pidOutput);
  MotorSpeed((int)(pidOutput * speedFactor));

  delay(10);
}

double pid(int error) {
  if (abs(error) <= tolerance) error = 0;
  double proportional = error;
  integral += error * dt;
  double differential = (error - prev_error) / dt;
  prev_error = error;

  return (Kp * proportional) + (Ki * integral) + (Kd * differential);
}

void countDegrees(){
  if (digitalRead(YELLOW_SIG) == digitalRead(BLUE_SIG)){
    degrees--;
  } else {
    degrees++;
  }
  if (degrees == 360 || degrees == -360){
    degrees = 0;
  }
}

void MotorSpeed(int speed) {
  if (speed > 100) speed = 100;
  if (speed < -100) speed = -100;

  analogWrite(EN, int(abs(speed)*2.55));

  if (speed > 0) {
    digitalWrite(O1, HIGH); // forwards
    digitalWrite(O2, LOW);
  } else {
    digitalWrite(O1, LOW); // backwards
    digitalWrite(O2, HIGH);
  }
}



