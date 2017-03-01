#include <ams_as5048b.h>
#include <AFMotor.h>

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
  pinMode(xendstop, INPUT);
  pinMode(yendstop, INPUT);
  
  // Put your setup code here, to run once:
  encoderX.begin();
  encoderY.begin();

  // Use these functions to set the direction that the encoder reads in
  encoderX.setClockWise(false);
  encoderY.setClockWise(false);

  
}

void moveX(int xtarget) {
  
}

void moveY(int ytarget) {
  
}

void zero() {
  // Move the X and Y stepper until the end stops are hit. 
  int xSwitch = digitalRead(endStopX);
  int ySwitch = digitalRead(endStopY);
  while(xSwitch == LOW || ySwitch == LOW) {
    if(xSwitch == LOW) {
      stepperX.move(1, FORWARD, MICROSTEP);      
    }
    if(ySwitch == LOW) {
      stepperY.move(1, FORWARD, MICROSTEP;
    }
    xSwitch = digitalRead(endStopX);
    ySwitch = digitalRead(endStopY);
  }
  
  encoderX.setZeroReg();
  encoderY.setZeroReg();
}

void readEncoderX() {
  angleX = encoderX.angleR(U_DEG);
  if(angleX > 0 && angleX < 180 && archiveangleX > 180 && archiveAngleX < 360) {
    turnX--;
  }
  else if(angleX > 180 && angleX < 360 && archiveAngleX > 0 && archiveAngleX < 180) {
    turnX++;  
  }

  archiveAngleX = angleX;
}

void readEncoderY() {
  angleY = encoderY.angleR(U_DEG);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
