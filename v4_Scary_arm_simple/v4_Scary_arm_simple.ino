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


void setup() {
  // put your setup code here, to run once 
  digitalWrite(stpmode0, LOW);
  digitalWrite(stpmode1, LOW);
  digitalWrite(stpmode2, LOW);
  
  Serial.begin(9600);
  Serial.println("READY");
  // Initialize PCA9685 PWM chip 
  pwm.begin();
  pwm.setPWMFreq(60);
  yield();
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

void moveServo0(int pos0) {
  int dir;
  if(pos0 > servoPos0) {
    dir = 1;
  } else {
    dir = -1;
  }
  while(pos0 != servoPos0) {
    servoPos0 += dir;
    pwm.setPWM(firstJointPin, 0, servoPos0);
    delay(MOVEDELAY);
  }
}

void moveServo1(int pos1) {
  int dir;
  if(pos1 > servoPos1) {
    dir = 1;
  } else {
    dir = -1;
  }
  while(pos1 != servoPos1) {
    servoPos1 += dir;
    pwm.setPWM(secondJointPin, 0, servoPos1);
    delay(MOVEDELAY);
  }
}

void moveServo2(int pos2) {
  int dir;
  if(pos2 > servoPos2) {
    dir = 1;
  } else {
    dir = -1;
  }
  while(pos2 != servoPos2) {
    servoPos2 += dir;
    pwm.setPWM(thirdJointPin, 0, servoPos2);
    delay(MOVEDELAY);
  }
}

void checkEnd() {
  if(Serial.read() == 'p') {
    lift(1500);   
    while(true) {
      
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0) {
    rotateStart();
    toStartHeight();
    clawOpen();
    moveServo0(436);
    moveServo1(575);
    moveServo2(155);
    delay(3000);
    lift(4975);
    while(Serial.read() != 'n') {
      
    }
    lift(0);
    delay(6000);
    ////////////////////////// INITIALIZATION
    moveServo0(535);
    moveServo1(514);
    moveServo2(531);
    delay(1000);
    rotateCustom(555);
    toCustomHeight(155);
    delay(6500);
    clawClosed();
    delay(2000);
    toStartHeight();
    delay(5000);
    ///////////////////////// FIRST PENNY STACK
    rotateDrop();
    moveServo0(360);
    moveServo1(110);
    moveServo2(432);
    delay(1000);
    toDropHeight();
    delay(2500);
    clawOpen();
    delay(1000);
    toStartHeight();
    rotatePickUp();
    //////////////////////// DROP FIRST
    delay(3000);
    moveServo0(370);
    moveServo1(400);
    moveServo2(279);
    delay(1000);
    rotatePickUp();
    clawOpen();
    toCustomHeight(168);
    delay(6000);
    clawClosed();
    delay(1000);
    toStartHeight();
    delay(6000);
    ///////////////////////// SECOND PENNY STACK
    rotateDrop();
    moveServo0(360);
    moveServo1(110);
    moveServo2(470);
    delay(1000);
    toDropHeight();
    delay(2500);
    clawOpen();
    delay(1000);
    toStartHeight();
    rotatePickUp();
    //////////////////////// DROP SECOND
    delay(3000);
    moveServo0(285);
    moveServo1(270);
    moveServo2(303);
    delay(1000);
    rotatePickUp();
    clawOpen();
    toCustomHeight(160);
    delay(5500);
    clawClosed();
    delay(2000);
    toStartHeight();
    delay(6000);
    //////////////////////// THIRD PENNY STACK
    rotateDrop();
    moveServo0(295);
    moveServo1(154);
    moveServo2(432);
    delay(1000);
    toDropHeight();
    delay(2500);
    clawOpen();
    delay(1000);
    toStartHeight();
    rotatePickUp();
    //////////////////////// DROP THIRD
    delay(3000);
    moveServo0(215);
    moveServo1(235);
    moveServo2(476);
    delay(1000);
    rotateCustom(556);
    clawOpen();
    toCustomHeight(150);
    delay(5500);
    clawClosed();
    delay(2000);
    toStartHeight();
    delay(6000);
    checkEnd();
    /////////////////////// FOURTH PENNY STACK
    rotateDrop();
    moveServo0(320);
    moveServo1(150);
    moveServo2(350);
    checkEnd();
    delay(1000);
    toDropHeight();
    delay(2500);
    clawOpen();
    delay(1000);
    toStartHeight();
    rotatePickUp();
    checkEnd();
    //////////////////////// DROP FOURTH
    delay(4000);
    moveServo0(210);
    moveServo1(185);
    moveServo2(190);
    checkEnd();
    delay(1000);
    rotatePickUp();
    clawOpen();
    toCustomHeight(140);
    checkEnd();
    delay(6500);
    clawClosed();
    delay(2000);
    toStartHeight();
    checkEnd();
    delay(6000); 
    ////////////////////// FIFTH PENNY STACK
    rotateDrop();
    moveServo0(235);
    moveServo1(245);
    moveServo2(349);
    checkEnd();
    delay(1000);
    toDropHeight();
    delay(2500);
    clawOpen();
    delay(1000);
    checkEnd();
    toStartHeight();
    delay(6000);
    rotatePickUp();
    checkEnd(); 
    //////////////////////// DROP FIFTH
    rotateStart();
    toStartHeight();
    clawOpen();
    moveServo0(436);
    moveServo1(570);
    moveServo2(155);
    
    while(Serial.read() != 'g') {
      
    }
    lift(3000);
    delay(2000000);
  }  
}
