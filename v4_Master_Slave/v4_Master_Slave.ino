#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!

int val1, val2, val3;
int count1 = 0;
int count2 = 0;
double pulsex = 450;
double pulsey = 500; 
double pulsez = 400;

const int firstJointPin = 0; // Joint A (Master is potPinA)
const int secondJointPin = 4; // Joint B (Master is potPinB)
const int thirdJointPin = 1; // Joint C (Master is potPinC)
const int winchServoPin = 5; // Winch Servo
const int rotatePin = 2; // Gripper Flip
const int clawPin = 3; // Gripper
const int stepperDirectionPin = 28; // Stepper direction control pin
const int stepperStepPin = 6; // Stepper step pulse pin

const int stpmode0 = 32;
const int stpmode1 = 33;
const int stpmode2 = 34;

const int MOVEDELAY = 8;

int servoPos0 = 435; // first joint
int servoPos1 = 555; // second joint
int servoPos2 = 170; // third joint
int servoPosWinch = 380; // 
int servoPosRotate = 350;
int servoPosClaw = 140;
int currentLiftPosition = 0;

char receivedChar;

int x = 0;

void setup() {
  // put your setup code here, to run once:
  digitalWrite(stpmode0, LOW);
  digitalWrite(stpmode1, LOW);
  digitalWrite(stpmode2, LOW);
  Serial.begin(9600);
  // Initialize PCA9685 PWM chip 
  pwm.begin();
  pwm.setPWMFreq(60);
  yield();
  Serial.println("s: STOP ARM MOVEMENT");
  Serial.println("g: BEGIN ARM MOVEMENT");
  Serial.println();
  Serial.println("i: Check Height");
  Serial.println("k: Drop Height");
  Serial.println("m: Pickup Height");
  Serial.println("j: Start Height");
  Serial.println();
  Serial.println("o: Claw Open");
  Serial.println("p: Claw Closed");
  Serial.println();
  Serial.println("n: Claw Rotation: Pickup");
  Serial.println("b: Claw Rotation: Drop");
  Serial.println("h: Claw Rotation: Start");
  Serial.println();
  Serial.println("q: Lift Up");
  Serial.println("a: Lift Halfway Up");
  Serial.println("z: Lift Down");
  
}

/////////////////////////////////////////////////////////////// 
void rotatePickUp() { pwm.setPWM(rotatePin, 0, 565); }
void rotateDrop() { pwm.setPWM(rotatePin, 0, 150); }
void rotateStart() { pwm.setPWM(rotatePin, 0, 350); }
void rotateCustom(int rotate) {
  pwm.setPWM(rotatePin, 0, rotate);
}
/////////////////////////////////////////////////////////////// 
void clawOpen() { pwm.setPWM(clawPin, 0, 140); }
void clawClosed() { pwm.setPWM(clawPin, 0, 330); }
/////////////////////////////////////////////////////////////// 
void toDropHeight() { pwm.setPWM(winchServoPin, 0, 300); }
void toPickUpHeight() { pwm.setPWM(winchServoPin, 0, 165); }
void toCheckHeight() { pwm.setPWM(winchServoPin, 0, 250); }
void toStartHeight() { pwm.setPWM(winchServoPin, 0, 380); }
void toCustomHeight(int height) {
  pwm.setPWM(winchServoPin, 0, height);
}
/////////////////////////////////////////////////////////////// 

void lift(int targetSteps) {
  int dir;
  if(currentLiftPosition > targetSteps) { 
    digitalWrite(stepperDirectionPin, 0); 
    dir = -1;
  }
  else if(currentLiftPosition < targetSteps) { 
    digitalWrite(stepperDirectionPin, 1); 
    dir = 1;
  }
  
  int stepStat = 0;
  while(currentLiftPosition != targetSteps) {
    currentLiftPosition+=dir;
    digitalWrite(stepperStepPin, stepStat);
    delayMicroseconds(750);
    if(stepStat == 0) { 
      stepStat = 1; 
    } 
    else { 
      stepStat = 0; 
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if(x) {
    val1 = analogRead(A2);            // reads the value of the potentiometer (value between 0 and 1023)
    pulsex = map(val1, 470, 120, 100, 650);
    val2 = analogRead(A3);            // reads the value of the potentiometer (value between 0 and 1023)
    pulsey = map(val2, 120, 470, 650, 100);
    val3 = analogRead(A4);            // reads the value of the potentiometer (value between 0 and 1023)
    pulsez = map(val3, 120, 470, 650, 100);
    
    pwm.setPWM(firstJointPin, 0, pulsex); //joint 1
    pwm.setPWM(secondJointPin, 0, pulsey); //joint 2
    pwm.setPWM(thirdJointPin, 0, pulsez); //joint 3
  }
  
  if(Serial.available() > 0) {
    receivedChar = Serial.read();
    
    if(receivedChar == 'i') { toCheckHeight(); }
    if(receivedChar == 'k') { toDropHeight(); }
    if(receivedChar == 'm') { toPickUpHeight(); }
    if(receivedChar == 'j') { toStartHeight(); }
    if(receivedChar == 'o') { clawOpen(); }
    if(receivedChar == 'p') { clawClosed(); }
    if(receivedChar == 'n') { rotatePickUp(); }
    if(receivedChar == 'b') { rotateDrop(); }
    if(receivedChar == 'h') { rotateStart(); }
    if(receivedChar == 'q') { 
      lift(4975); 
    }
    if(receivedChar == 'a') {
      lift(1600); 
    }
    if(receivedChar == 'z') { 
      lift(0); 
    }
    if(receivedChar == 's') { 
      x = 0; 
    }
    if(receivedChar == 'g') { 
      x = 1; 
    }
  }
}
