// Pins:
#define EN_INDEX_MCP 13
#define EN_INDEX_IP 12
#define EN_THUMB_MCP 11
#define EN_THUMB_IP 10
#define EN_MID_MCP 9
#define EN_MID_IP 8

#define O1_INDEX_MCP 22
#define O2_INDEX_MCP 23
#define O1_INDEX_IP 24
#define O2_INDEX_IP 25
#define O1_THUMB_MCP 26
#define O2_THUMB_MCP 27
#define O1_THUMB_IP 28
#define O2_THUMB_IP 29
#define O1_MID_MCP 30
#define O2_MID_MCP 31
#define O1_MID_IP 32
#define O2_MID_IP 33

#define YSIG_INDEX_MCP 2 // interrupt pin
#define BSIG_INDEX_MCP 53 // digital pin
#define YSIG_INDEX_IP 3
#define BSIG_INDEX_IP 52
#define YSIG_THUMB_MCP 18
#define BSIG_THUMB_MCP 51
#define YSIG_THUMB_IP 19
#define BSIG_THUMB_IP 50
#define YSIG_MID_MCP 20
#define BSIG_MID_MCP 49
#define YSIG_MID_IP 21
#define BSIG_MID_IP 48

#define INPUT_INDEX A1
#define INPUT_THUMB A3
#define INPUT_MID A4

#define MCP_MAX_ANGLE 90

// PID Tuning
#define Kp 1
#define Ki 0.9
#define Kd 0.05

#define TOLERANCE 0

#define SPEEDFACTOR 1.3

// Class Definitions
class Joint {
  public:
    volatile int angle = 0;

    Joint(uint8_t EN, uint8_t O1, uint8_t O2, uint8_t YSIG, uint8_t BSIG) {
      this->EN = EN;
      this->O1 = O1;
      this->O2 = O2;
      this->YSIG = YSIG;
      this->BSIG = BSIG;
    }

    double pid(int error) {
      now = millis();
      dt  = (now - last_time) / 1000.00;
      last_time = now;

      if (abs(error) <= TOLERANCE) error = 0;
      proportional = error;
      integral += error * dt;
      differential = (error - prev_error) / dt;
      prev_error = error;

      return (Kp * proportional) + (Ki * integral) + (Kd * differential);
    }

    void motorSpeed(int velocity) {
      if (velocity > 100) velocity = 100;
      if (velocity < -100) velocity = -100;

      analogWrite(EN, int(abs(velocity)*2.55));

      if (velocity > 0) {
        digitalWrite(O1, HIGH); // forwards
        digitalWrite(O2, LOW);
      } else {
        digitalWrite(O1, LOW); // backwards
        digitalWrite(O2, HIGH);
      }
    }
    
    void countDegrees(){
      if (digitalRead(YSIG) == digitalRead(BSIG)){
        angle--;
      } else {
        angle++;
      }
      if (angle == 360 || angle == -360){
        angle = 0;
      }
    }

  private:
    double integral = 0;
    double prev_error = 0;
    double last_time = 0;
    double now;
    double dt;
    double proportional;
    double differential;
    
    uint8_t EN;
    uint8_t O1;
    uint8_t O2;
    uint8_t YSIG;
    uint8_t BSIG;
};

class Finger {
  public:
    Joint* IP;
    Joint* MCP;

    Finger(uint8_t IN, uint8_t IP_EN, uint8_t IP_O1, uint8_t IP_O2, uint8_t IP_YSIG, uint8_t IP_BSIG, uint8_t MCP_EN, uint8_t MCP_O1, uint8_t MCP_O2, uint8_t MCP_YSIG, uint8_t MCP_BSIG) {
      this->IN = IN;
      initPotVal = analogRead(IN);

      IP = new Joint(IP_EN, IP_O1, IP_O2, IP_YSIG, IP_BSIG);
      MCP = new Joint(MCP_EN, MCP_O1, MCP_O2, MCP_YSIG, MCP_BSIG);
    }

    tick() {
      potVal = analogRead(IN);
      
      setpoint = (potVal - initPotVal) / 5;
      setpoint = round((float)setpoint / 10)*10;
      // setpoint measured in degrees from starting position

      if (setpoint > 120) {
        setpoint = 120;
      }

      if (setpoint > MCP_MAX_ANGLE) {
        IPsetpoint = setpoint - MCP_MAX_ANGLE;
        MCPsetpoint = MCP_MAX_ANGLE;
      } else if (setpoint < -MCP_MAX_ANGLE) {
        IPsetpoint = setpoint + MCP_MAX_ANGLE;
        MCPsetpoint = -MCP_MAX_ANGLE;
      } else {
        IPsetpoint = 0;
        MCPsetpoint = setpoint;
      }

      pidOutput = IP->pid(IPsetpoint - IP->angle);
      IP->motorSpeed((int)(pidOutput * SPEEDFACTOR));

      pidOutput = MCP->pid(MCPsetpoint - MCP->angle);
      MCP->motorSpeed((int)(pidOutput * SPEEDFACTOR));
    }

  private:
    uint8_t IN;
    uint8_t initPotVal;
    int potVal;
    int setpoint;
    int IPsetpoint;
    int MCPsetpoint;
    double pidOutput;
};

Finger* index;
Finger* thumb;
Finger* middle;

void setup() {
  Serial.begin(115200);
  while(!Serial);

  pinMode(EN_INDEX_IP, OUTPUT);
  pinMode(EN_INDEX_MCP, OUTPUT);

  pinMode(O1_INDEX_IP, OUTPUT);
  pinMode(O2_INDEX_IP, OUTPUT);
  pinMode(O1_INDEX_MCP, OUTPUT);
  pinMode(O2_INDEX_MCP, OUTPUT);

  pinMode(YSIG_INDEX_IP, INPUT);
  pinMode(BSIG_INDEX_IP, INPUT);  
  pinMode(YSIG_INDEX_MCP, INPUT);
  pinMode(BSIG_INDEX_MCP, INPUT);
  
  pinMode(INPUT_INDEX, INPUT);

  pinMode(EN_THUMB_IP, OUTPUT);
  pinMode(EN_THUMB_MCP, OUTPUT);

  pinMode(O1_THUMB_IP, OUTPUT);
  pinMode(O2_THUMB_IP, OUTPUT);
  pinMode(O1_THUMB_MCP, OUTPUT);
  pinMode(O2_THUMB_MCP, OUTPUT);

  pinMode(YSIG_THUMB_IP, INPUT);
  pinMode(BSIG_THUMB_IP, INPUT);  
  pinMode(YSIG_THUMB_MCP, INPUT);
  pinMode(BSIG_THUMB_MCP, INPUT);
  
  pinMode(INPUT_THUMB, INPUT);

  pinMode(EN_MID_IP, OUTPUT);
  pinMode(EN_MID_MCP, OUTPUT);

  pinMode(O1_MID_IP, OUTPUT);
  pinMode(O2_MID_IP, OUTPUT);
  pinMode(O1_MID_MCP, OUTPUT);
  pinMode(O2_MID_MCP, OUTPUT);

  pinMode(YSIG_MID_IP, INPUT);
  pinMode(BSIG_MID_IP, INPUT);  
  pinMode(YSIG_MID_MCP, INPUT);
  pinMode(BSIG_MID_MCP, INPUT);
  
  pinMode(INPUT_MID, INPUT);

  index = new Finger(INPUT_INDEX, EN_INDEX_IP, O1_INDEX_IP, O2_INDEX_IP, YSIG_INDEX_IP, BSIG_INDEX_IP, EN_INDEX_MCP, O1_INDEX_MCP, O2_INDEX_MCP, YSIG_INDEX_MCP, BSIG_INDEX_MCP);
  thumb = new Finger(INPUT_THUMB, EN_THUMB_IP, O1_THUMB_IP, O2_THUMB_IP, YSIG_THUMB_IP, BSIG_THUMB_IP, EN_THUMB_MCP, O1_THUMB_MCP, O2_THUMB_MCP, YSIG_THUMB_MCP, BSIG_THUMB_MCP);
  middle = new Finger(INPUT_MID, EN_MID_IP, O1_MID_IP, O2_MID_IP, YSIG_MID_IP, BSIG_MID_IP, EN_MID_MCP, O1_MID_MCP, O2_MID_MCP, YSIG_MID_MCP, BSIG_MID_MCP);

  attachInterrupt(digitalPinToInterrupt(YSIG_INDEX_MCP), index_MCP_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(YSIG_INDEX_IP), index_IP_ISR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(YSIG_THUMB_MCP), thumb_MCP_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(YSIG_THUMB_IP), thumb_IP_ISR, CHANGE);

  attachInterrupt(digitalPinToInterrupt(YSIG_MID_MCP), middle_MCP_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(YSIG_MID_IP), middle_IP_ISR, CHANGE);

  Serial.println("hello world");
}

void loop() {
  index->tick();
  thumb->tick();
  middle->tick();

  delay(10);
}

void index_MCP_ISR() {
  index->MCP->countDegrees();
}

void index_IP_ISR() {
  index->IP->countDegrees();
}

void thumb_MCP_ISR() {
  thumb->MCP->countDegrees();
}

void thumb_IP_ISR() {
  thumb->IP->countDegrees();
}

void middle_MCP_ISR() {
  middle->MCP->countDegrees();
}

void middle_IP_ISR() {
  middle->IP->countDegrees();
}