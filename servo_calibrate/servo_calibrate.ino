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

const int MOVEDELAY = 15;

int servoPos0 = 250; // first joint
int servoPos1 = 560; // second joint
int servoPos2 = 170; // third joint
int servoPosWinch = 380; // 
int servoPosRotate = 350;
int servoPosClaw = 140;
int currentLiftPosition = 0;

char receivedChar;

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
/////////////////////////////////////////////////////////////// 
void clawOpen() { pwm.setPWM(clawPin, 0, 140); }
void clawClosed() { pwm.setPWM(clawPin, 0, 340); }
/////////////////////////////////////////////////////////////// 
void toDropHeight() { pwm.setPWM(winchServoPin, 0, 300); }
void toPickUpHeight() { pwm.setPWM(winchServoPin, 0, 170); }
void toCheckHeight() { pwm.setPWM(winchServoPin, 0, 250); }
void toStartHeight() { pwm.setPWM(winchServoPin, 0, 380); }
/////////////////////////////////////////////////////////////// 

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0) {
    receivedChar = Serial.read();
    if(receivedChar == 'q') {
      servoPos0+=5;
      pwm.setPWM(firstJointPin,0, servoPos0); 
    }
    if(receivedChar == 'a') {
      servoPos0-=5;
      pwm.setPWM(firstJointPin,0, servoPos0); 
    }
    if(receivedChar == 'w') {
      servoPos1+=5;
      pwm.setPWM(secondJointPin,0, servoPos1); 
    }
    if(receivedChar == 's') {
      servoPos1-=5;
      pwm.setPWM(secondJointPin,0, servoPos1); 
    }
    if(receivedChar == 'e') {
      servoPos2+=5;
      pwm.setPWM(thirdJointPin,0, servoPos2); 
    }
    if(receivedChar == 'd') {
      servoPos2-=5;
      pwm.setPWM(thirdJointPin,0, servoPos2); 
    }
    if(receivedChar == 'i') { // WINCH
      toDropHeight();
    }
    if(receivedChar == 'k') {
      toCheckHeight();
    }
    if(receivedChar == 'm') {
      toPickUpHeight();
    }
    if(receivedChar == 'j') {
      toStartHeight();
    }
    if(receivedChar == 'o') { // CLAW
      clawOpen();
    }
    if(receivedChar == 'p') {
      clawClosed();
    }
    if(receivedChar == 'n') { // ROTATE
      rotatePickUp();
    }
    if(receivedChar == 'b') {
      rotateDrop();
    }
    if(receivedChar == 'h') {
      rotateStart();
    }
    Serial.print(servoPos0);
    Serial.print(", ");
    Serial.print(servoPos1);
    Serial.print(", ");
    Serial.println(servoPos2);
  }
}
