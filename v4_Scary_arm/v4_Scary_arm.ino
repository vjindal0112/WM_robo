#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define FWD 0
#define REV 1

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

int servo0Pos = 150;
int servo1Pos = 150;
int servo2Pos = 150;
int MOVEDELAY = 12;
char receivedChar;

const int potPinA = A2;
const int potPinB = A3;
const int potPinC = A4;
const int servoChannelA = 0; // Joint 1 (Master is potPinA)
const int servoChannelB = 1; // Joint 2 (Master is potPinB)
const int servoChannelC = 4; // Joint 3 (Master is potPinC)
const int servoChannelD = 3; // Winch Servo
const int servoChannelE = 2; // Gripper Flip
const int servoChannelF = 5; // Gripper

const int stepperDirectionC = 28;
const int stepperStepC = 6;

void setup() {
  Serial.begin(9600);
  Serial.println("WMHS Robot Arm");
  Serial.println("(C) 2017 Varun Jindal, Nestor Tkachenko, and David Cutting");
  Serial.println();
  Serial.println("Go get e'm, boys!");

  // Initialize PWM chip 
  pwm.begin();
  pwm.setPWMFreq(60);
  yield();
}

void lift(int dir) {
  int stepStat = 0;
  digitalWrite(stepperDirectionC, dir);
  for(int i = 2500; i > 0; i--) {
    digitalWrite(stepperStepC, stepStat);
    delayMicroseconds(600);
    if(stepStat == 0) {
      stepStat = 1;
    }
    else if(stepStat == 1) {
      stepStat = 0;
    }
  }
}

void moveServos(int pos0, int pos1, int pos2) {
  int dir0;
  int dir1;
  int dir2; 
  
  if(servo0Pos < pos0) {
    dir0 = 1;
  } else {
    dir0 = -1;
  }
  if(servo1Pos < pos1) {
    dir1 = 1;
  } else {
    dir1 = -1;
  }
  if(servo2Pos < pos2) {
    dir2 = 1;
  } else {
    dir2 = -1;
  }  
  while(servo0Pos != pos0) {
    servo0Pos+=dir0;
    pwm.setPWM(servoChannelA, 0, servo0Pos);
    delay(MOVEDELAY);
  }
  while(servo1Pos != pos1) {
    servo1Pos+=dir1;
    pwm.setPWM(servoChannelB, 0, servo1Pos);
    delay(MOVEDELAY);
  }
  while(servo2Pos != pos2) {
    servo2Pos+=dir2;
    pwm.setPWM(servoChannelC, 0, servo2Pos);
    delay(MOVEDELAY);
  }
}

// set positions
void rotatePickUp() {
  pwm.setPWM(servoChannelE, 0, 150); //rotate (570 up, 150 down)
}

void rotateDrop() {
  pwm.setPWM(servoChannelE, 0, 570); //rotate (570 up, 150 down)
}

void clawOpen() {
  pwm.setPWM(servoChannelF, 0, 140); //claw (530 closed, 300 open)
}

void clawClosed() {
  pwm.setPWM(servoChannelF, 0, 300); //claw (530 closed, 300 open)
}

void toDropHeight() {
  pwm.setPWM(servoChannelD, 0, 300); // Winch Servo
}

void toPickUpHeight() {
  pwm.setPWM(servoChannelD, 0, 200); // Winch Servo
}

void toCheckHeight() {
  pwm.setPWM(servoChannelD, 0, 250); // Winch Servo
}

void toBeginHeight() {
  pwm.setPWM(servoChannelD, 0, 400); // Winch Servo
}

void fineTune() {
  while(receivedChar != 'g') {
    receivedChar = Serial.read();
    if(receivedChar == 'w') {
      moveServos(servo0Pos + 1, servo1Pos, servo2Pos);
    }
    if(receivedChar == 's') {
      moveServos(servo0Pos - 1, servo1Pos, servo2Pos);
    }
    if(receivedChar == 'r') {
      moveServos(servo0Pos, servo1Pos + 1, servo2Pos);
    }
    if(receivedChar == 'f') {
      moveServos(servo0Pos, servo1Pos - 1, servo2Pos);
    }
    if(receivedChar == 'y') {
      moveServos(servo0Pos, servo1Pos, servo2Pos + 1);
    }
    if(receivedChar == 'h') {
      moveServos(servo0Pos, servo1Pos, servo2Pos - 1);
    }
    if(receivedChar == 'o') {
      lift(1);
    }
    if(receivedChar == 'l') {
      lift(0);
    }
  }
}

void manualcontrol() {
  
}

void loop() {
  /*
  clawOpen();
  toDropHeight();
  delay(2000);
  clawClosed();
  toCheckHeight();
  delay(2000);
  */
  pwm.setPWM(servoChannelA, 0, 150);
  delay(2000);
  pwm.setPWM(servoChannelA, 0, 200);
  delay(2000);
  pwm.setPWM(servoChannelB, 0, 150);
  delay(2000);
  pwm.setPWM(servoChannelB, 0, 200);
  delay(2000);
  pwm.setPWM(servoChannelC, 0, 150);
  delay(2000);
  pwm.setPWM(servoChannelC, 0, 200);
  delay(2000);
  
}
