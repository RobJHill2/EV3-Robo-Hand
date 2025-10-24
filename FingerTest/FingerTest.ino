// Pins:
#define EN_INDEX_MCP 13
#define EN_INDEX_IP 12

#define O1_INDEX_MCP 22
#define O2_INDEX_MCP 23
#define O1_INDEX_IP 24
#define O2_INDEX_IP 25

#define YSIG_INDEX_MCP 2 // interrupt bool
#define BSIG_INDEX_MCP 53 // bool
#define YSIG_INDEX_IP 3
#define BSIG_INDEX_IP 52

#define INPUT_INDEX A1

// PID Tuning
#define Kp 1
#define Ki 0.9
#define Kd 0.05

#define tolerance 0

#define speedFactor 1.3

// Class Definitions

class Joint {
  public:
  private:
}

class Finger {
  public:
  Finger() {
    initPotVal = analogRead(INPUT_INDEX);
  };
  private:
  int initPotVal;
  Joint MCP;
  Joint IP;
};




void setup() {
  Serial.begin(115200);
  while(!Serial);

  pinMode(EN_INDEX_MCP, OUTPUT);
  pinMode(EN_INDEX_IP, OUTPUT);

  pinMode(O1_INDEX_MCP, OUTPUT);
  pinMode(O2_INDEX_IP, OUTPUT);
  pinMode(O1_INDEX_IP, OUTPUT);
  pinMode(O2_INDEX_IP, OUTPUT);

  pinMode(YSIG_INDEX_MCP, INPUT);
  pinMode(BSIG_INDEX_MCP, INPUT);
  pinMode(YSIG_INDEX_IP, INPUT);
  pinMode(BSIG_INDEX_IP, INPUT);

  pinMode(INPUT_INDEX, INPUT);

  Finger index;

  // attachInterrupt();

  Serial.println("hello world";)
}

void loop() {
  // put your main code here, to run repeatedly:

}
