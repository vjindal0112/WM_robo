#include <Wire.h>
#include <AFMotor.h>
#include <ams_as5048b.h>

//unit consts
#define U_RAW 1
#define U_DEG 3
#define U_RAD 4

AMS_AS5048B encoderX(0x40);
AMS_AS5048B encoderY(0x44);
AF_Stepper stepperX(200, 1); // FIX PORTS
AF_Stepper stepperY(200, 2);

const int endStopX = A0;
const int endStopY = A1;
int xpos = 0;
int ypos = 0;
float angleX = 0;
float angleY = 0;
float archiveAngleX = 0;
float archiveAngleY = 0;
int turnX = 0;
int turnY = 0;

void setup() {
  pinMode(endStopX, INPUT);
  pinMode(endStopY, INPUT);
  
  // Put your setup code here, to run once:
  encoderX.begin();
  encoderY.begin();

  // Use these functions to set the direction that the encoder reads in
  encoderX.setClockWise(false);
  encoderY.setClockWise(false);

  
}

void moveX(int xtarget) {
  int xdiff = xtarget - xpos;
  if (xdiff > 0) {
    while (xpos < xtarget) {
      stepperX.step(1, FORWARD, MICROSTEP); // FIX: CHECK DIRECTION
      readEncoderX();
    }
  } 
  else if (xdiff < 0) {
    while (xpos > xtarget) {
      stepperX.step(1, BACKWARD, MICROSTEP); // FIX: CHECK DIRECTION
      readEncoderX();
    }
  }
  
}

void moveY(int ytarget) {
  int ydiff = ytarget - ypos;
  if (ydiff > 0) {
    while (ypos < ytarget) {
      stepperY.step(1, FORWARD, MICROSTEP); // FIX: CHECK DIRECTION
      readEncoderY();
    }
  } 
  else if (ydiff < 0) {
    while (ypos > ytarget) {
      stepperY.step(1, BACKWARD, MICROSTEP); // FIX: CHECK DIRECTION
      readEncoderY();
    }
  }
}

/*inline void moveLogic(AF_Stepper stepper, int target, int pos) {
  int xdiff = xtarget - xpos;
  if (xdiff > 0) {
    while (xpos < xtarget) {
      stepperX.step(1, FORWARD, MICROSTEP);
      readEncoderX();
    }
  } 
  else if (xdiff < 0) {
    xdir = BACKWARD;
    while (xpos > xtarget) {
      stepperX.step(1, BACKWARD, MICROSTEP);
      readEncoderX();
    }
  }
} */

void zero() {
  // Move the X and Y stepper until the end stops are hit. 
  int xSwitch = digitalRead(endStopX);
  int ySwitch = digitalRead(endStopY);
  while(xSwitch == LOW || ySwitch == LOW) { // FIX: CHECK IF LOW OR HIGH
    if(xSwitch == LOW) {
      stepperX.step(1, FORWARD, MICROSTEP);      
    }
    if(ySwitch == LOW) {
      stepperY.step(1, FORWARD, MICROSTEP);
    }
    xSwitch = digitalRead(endStopX);
    ySwitch = digitalRead(endStopY);
  }
  
  encoderX.setZeroReg();
  encoderY.setZeroReg();
}

void readEncoderX() {
  angleX = encoderX.angleR(U_DEG);
  if(angleX > 0 && angleX < 100 && archiveAngleX > 260 && archiveAngleX < 360) {
    turnX--;
  }
  else if(angleX > 260 && angleX < 360 && archiveAngleX > 0 && archiveAngleX < 100) {
    turnX++;  
  }
  xpos = (360 * turnX) + angleX;
  archiveAngleX = angleX;
}

void readEncoderY() {
  angleY = encoderY.angleR(U_DEG);
  if(angleY > 0 && angleY < 100 && archiveAngleY > 260 && archiveAngleY < 360) {
    turnY--;
  }
  else if(angleY > 260 && angleY < 360 && archiveAngleY > 0 && archiveAngleY < 100) {
    turnY++;  
  }
  ypos = (360 * turnY) + angleY;
  archiveAngleY = angleY;
}

/*inline void degToPos(AMS_AS5048B encoder, int angle, int archiveAngle, int turn, int pos) {
  if(angle > 0 && angle < 100 && archiveAngle > 260 && archiveAngle < 360) {
    turn--;
  }
  else if(angle > 260 && angle < 360 && archiveAngle > 0 && archiveAngle < 100) {
    turn++;  
  }
  pos = (360 * turn) + angle;
} */

void loop() {
  // put your main code here, to run repeatedly:

}
