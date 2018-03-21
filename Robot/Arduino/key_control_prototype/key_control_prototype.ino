#define L1_F 2
#define L1_B 3
#define R1_F 4
#define R1_B 5

#define L2_F 6
#define L2_B 7
#define R2_F 8
#define R2_B 9

#define L3_F 10
#define L3_B 11
#define R3_F 12
#define R3_B 13

const int KEY_DELAY = 250;
const int FORWARD_SPEED = 130;
const int TURN_SPEED = 150;

int inpt;

void setup() {

	pinMode(L1_F, OUTPUT);
	pinMode(L1_B, OUTPUT);
	pinMode(R1_F, OUTPUT);
	pinMode(R1_B, OUTPUT);
	pinMode(L2_F, OUTPUT);
	pinMode(L2_B, OUTPUT);
	pinMode(R2_F, OUTPUT);
	pinMode(R2_B, OUTPUT);
	pinMode(L3_F, OUTPUT);
	pinMode(L3_B, OUTPUT);
	pinMode(R3_F, OUTPUT);
	pinMode(R3_B, OUTPUT);

    digitalWrite(L1_F, LOW);
    digitalWrite(L1_B, LOW);
    digitalWrite(R1_F, LOW);
    digitalWrite(R1_B, LOW);

    digitalWrite(L2_F, LOW);
    digitalWrite(L2_B, LOW);
    digitalWrite(R2_F, LOW);
    digitalWrite(R2_B, LOW);

    digitalWrite(L3_F, LOW);
    digitalWrite(L3_B, LOW);
    digitalWrite(R3_F, LOW);
    digitalWrite(R3_B, LOW);

  Serial.begin(9600);
}

void loop() {

  if (Serial.available()) // Non-blocking
  {
    inpt = Serial.read();
    switch(inpt) // switch between WASD in ASCII
    {
      case 119: // w
        Serial.println("forward");
        analogWrite(L1_F, FORWARD_SPEED);
        analogWrite(L2_F, FORWARD_SPEED);
        analogWrite(L3_F, FORWARD_SPEED);
        analogWrite(R1_F, FORWARD_SPEED);
        analogWrite(R2_F, FORWARD_SPEED);
        analogWrite(R3_F, FORWARD_SPEED);
        delay(KEY_DELAY);
        analogWrite(L1_F, 0);
        analogWrite(L2_F, 0);
        analogWrite(L3_F, 0);
        analogWrite(R1_F, 0);
        analogWrite(R2_F, 0);
        analogWrite(R3_F, 0);
        break;
      case 97: //a
        Serial.println("left");
        analogWrite(L1_F, TURN_SPEED);
        analogWrite(L2_F, TURN_SPEED);
        analogWrite(L3_F, TURN_SPEED);
        analogWrite(R1_B, TURN_SPEED);
        analogWrite(R2_B, TURN_SPEED);
        analogWrite(R3_B, TURN_SPEED);
        delay(KEY_DELAY);
        analogWrite(L1_F, 0);
        analogWrite(L2_F, 0);
        analogWrite(L3_F, 0);
        analogWrite(R1_B, 0);
        analogWrite(R2_B, 0);
        analogWrite(R3_B, 0);
        break;
      case 115: //s
        Serial.println("reverse");
        analogWrite(L1_B, FORWARD_SPEED);
        analogWrite(L2_B, FORWARD_SPEED);
        analogWrite(L3_B, FORWARD_SPEED);
        analogWrite(R1_B, FORWARD_SPEED);
        analogWrite(R2_B, FORWARD_SPEED);
        analogWrite(R3_B, FORWARD_SPEED);
        delay(KEY_DELAY);
        analogWrite(L1_B, 0);
        analogWrite(L2_B, 0);
        analogWrite(L3_B, 0);
        analogWrite(R1_B, 0);
        analogWrite(R2_B, 0);
        analogWrite(R3_B, 0);
        break;
      case 100: //d
        Serial.println("right");
        analogWrite(L1_B, TURN_SPEED);
        analogWrite(L2_B, TURN_SPEED);
        analogWrite(L3_B, TURN_SPEED);
        analogWrite(R1_F, TURN_SPEED);
        analogWrite(R2_F, TURN_SPEED);
        analogWrite(R3_F, TURN_SPEED);
        delay(KEY_DELAY);
        analogWrite(L1_B, 0);
        analogWrite(L2_B, 0);
        analogWrite(L3_B, 0);
        analogWrite(R1_F, 0);
        analogWrite(R2_F, 0);
        analogWrite(R3_F, 0);
        break;
    }
  }   
}
