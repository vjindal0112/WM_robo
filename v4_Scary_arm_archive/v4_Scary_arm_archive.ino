#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

int servoPosA = 150; // Current servo A position (pulse length)
int servoPosB = 150; // Current servo B position (pulse length)
int servoPosC = 150; // Current servo C position (pulse length)


int currentLiftPosition = 0;

const int potPinA = A2; // Potentiometer A (Master for Joint A)
const int potPinB = A4; // Potentiometer B (Master for Joint B)
const int potPinC = A3; // Potentiometer C (Master for Joint C)
const int servoChannelA = 0; // Joint A (Master is potPinA)
const int servoChannelB = 1; // Joint B (Master is potPinB)
const int servoChannelC = 4; // Joint C (Master is potPinC)
const int servoChannelD = 5; // Winch Servo
const int servoChannelE = 2; // Gripper Flip
const int servoChannelF = 3; // Gripper
const int stepperDirectionC = 28; // Stepper direction control pin
const int stepperStepC = 6; // Stepper step pulse pin

// Pin numbers for A, B, and C (not D) step mode selection
const int stpmode0 = 32;
const int stpmode1 = 33;
const int stpmode2 = 34;

char serialInputChar = 's'; // Serial read variable
char controlMode = 's'; // Initialize control mode to "Stop"
char returnToControlMode = 's'; // Initialize return-to control mode to "Stop"

int potValA = 0;
int potValB = 0;
int potValC = 0;

int servoacurrcount = 150;
int servoatargetcount = 150;
int servoaswitch = 0;
long servoatimer = 0;

int servobcurrcount = 150;
int servobtargetcount = 150;
int servobswitch = 0;
long servobtimer = 0;

int servoccurrcount = 150;
int servoctargetcount = 150;
int servocswitch = 0;
long servoctimer = 0;

const int servoadelay = 50; // Servo A delay variables, in milliseconds
const int servobdelay = 50; // Servo B delay variables, in milliseconds
const int servocdelay = 50; // Servo C delay variables, in milliseconds

void setup() {
  // Set step mode for lift motor (Full Step Mode)
  digitalWrite(stpmode0, LOW);
  digitalWrite(stpmode1, LOW);
  digitalWrite(stpmode2, LOW);
  
  Serial.begin(250000); //Begin serial line (115200 baud rate)

  // Print initialization lines
  Serial.println("WMHS Robot Arm");
  Serial.println("(C) 2017 Varun Jindal, Nestor Tkachenko, and David Cutting");
  Serial.println();
  Serial.println("Make sure that the lift motor is in the down position before zeroing/starting");

  // Initialize PCA9685 PWM chip 
  pwm.begin();
  pwm.setPWMFreq(60);
  yield();
}

void autoRun() {
  // Put your action here
  if(checkserial('g')) { return; }
}

int checkserial(char character) {
  if(Serial.available()) {
    serialInputChar = Serial.read();
    if(serialInputChar = character) {
      return 0;
    } else {
      return 1; 
    }
  } else {
    return 0;
  }
}

void loop() {
  if(Serial.available()) {
    controlMode = Serial.read();
    Serial.println(controlMode);
  }
  
  moveservoa();
  moveservob();
  moveservoc();
  
  switch(controlMode) {
    ///////////////////////////////////////////////////////////////
    case 's':         // "Stop" 
      returnToControlMode = 's';
      break;
    case 'g':         // Auto Run ("Go")
      autoRun();
      controlMode = 's';
      break;
    ///////////////////////////////////////////////////////////////  
    case 'a':         // Joint A CW/CCW (TODO: Determine Direction)
      servoatargetcount++;
      controlMode = returnToControlMode;
      break;
    case 'b':         // Joint A CW/CCW (TODO: Determine Direction)
      servoatargetcount--;
      controlMode = returnToControlMode;
      break;
    /////////////////////////////////////////////////////////////// 
    case 'c':         // Joint B CW/CCW (TODO: Determine Direction)
      servobtargetcount++;
      controlMode = returnToControlMode;
      break;
    case 'd':         // Joint B CW/CCW (TODO: Determine Direction)
      servobtargetcount--;
      controlMode = returnToControlMode;
      break;
    /////////////////////////////////////////////////////////////// 
    case 'e':         // Joint C CW/CCW (TODO: Determine Direction)
      servoctargetcount++;
      controlMode = returnToControlMode;
      break;
    case 'f':         // Joint C CW/CCW (TODO: Determine Direction)
      servoctargetcount--;
      controlMode = returnToControlMode;
      break;
    /////////////////////////////////////////////////////////////// 
    case 'p':         // Servo Lift DROP
      toDropHeight();
      controlMode = returnToControlMode;
      break;  
    case 'h':         // Servo Lift PICK UP
      toPickUpHeight();
      controlMode = returnToControlMode;
      break;
    case 'i':         // Servo Lift CHECK
      toCheckHeight();
      controlMode = returnToControlMode;
      break;
    case 'j':         // Servo Lift START
      toBeginHeight();
      controlMode = returnToControlMode;
      break;
    /////////////////////////////////////////////////////////////// 
    case 'k':         // Gripper OPEN
      clawOpen();
      controlMode = returnToControlMode;
      break;
    case 'l':         // Gripper CLOSE
      clawClosed();
      controlMode = returnToControlMode;
      break;
    /////////////////////////////////////////////////////////////// 
    case 'm':         // Gripper Flip PICK UP
      rotatePickUp();
      controlMode = returnToControlMode;
      break;
    case 'n':         // Gripper Flip DROP
      rotateDrop();
      controlMode = returnToControlMode;
      break;
    /////////////////////////////////////////////////////////////// 
    case 'o':         // Potentiometer Arm Control
      servoatargetcount = map(analogRead(potPinA), 470, 120, 100, 650);
      servobtargetcount = map(analogRead(potPinB), 470, 120, 100, 650);
      servoctargetcount = map(analogRead(potPinC), 120, 470, 100, 650); 
      returnToControlMode = 'o';
      break;
    ///////////////////////////////////////////////////////////////
    case 'x':         // Move lift mechanism to up position
      lift(90);
      controlMode = returnToControlMode;
      break;
    case 'y':         // Move lift mechanism to halfway position
      lift(45);
      controlMode = returnToControlMode;
      break;
    case 'z':         // Move lift mechanism to down position
      lift(0);
      controlMode = returnToControlMode;
      break;
    ////////////////////////////////////////////////////////////////
  } 
}

void lift(int targetDeg) {
  int dir;
  if(currentLiftPosition > targetDeg) { 
    digitalWrite(stepperDirectionC, 0); 
  }
  else if(currentLiftPosition < targetDeg) { 
    digitalWrite(stepperDirectionC, 1); 
  }
  currentLiftPosition = targetDeg;
  
  int stepStat = 0;
  for(int i = map(targetDeg, 0, 90, 0, 3100); i > 0; i--) {
    digitalWrite(stepperStepC, stepStat);
    delayMicroseconds(750);
    if(stepStat == 0) { 
      stepStat = 1; 
    } 
    else { 
      stepStat = 0; 
    }
  }
}

void moveservoa() {
  switch (servoaswitch) {
    case 0:
      if(servoatargetcount != servoacurrcount) {
        servoaswitch = 1;
        servoatimer = micros();
      }
      break;
    case 1:
      if(micros() >= (servoatimer + servoadelay)) {
        if(servoacurrcount > servoatargetcount) { servoacurrcount--; }
        else if(servoacurrcount < servoatargetcount) { servoacurrcount++; }
        else { servoaswitch = 0; } 
        pwm.setPWM(servoChannelA, 0, servoacurrcount);
        servoatimer = micros();
      }
      break;
  }
}

void moveservob() {
  switch (servobswitch) {
    case 0:
      if(servobtargetcount != servobcurrcount) {
        servobswitch = 1;
        servobtimer = micros();
      }
      break;
    case 1:
      if(micros() >= (servobtimer + servobdelay)) {
        if(servobcurrcount > servobtargetcount) { servobcurrcount--; }
        else if(servobcurrcount < servobtargetcount) { servobcurrcount++; }
        else { servobswitch = 0; } 
        pwm.setPWM(servoChannelB, 0, servobcurrcount);
        servobtimer = micros();
      }
      break;
  }
}

void moveservoc() {
  switch (servocswitch) {
    case 0:
      if(servoctargetcount != servoccurrcount) {
        servocswitch = 1;
        servoctimer = micros();
      }
      break;
    case 1:
      if(micros() >= (servoctimer + servocdelay)) {
        if(servoccurrcount > servoctargetcount) { servoccurrcount--; }
        else if(servoccurrcount < servoctargetcount) { servoccurrcount++; }
        else { servocswitch = 0; } 
        pwm.setPWM(servoChannelC, 0, servoccurrcount);
        servoctimer = micros();
      }
      break;
  }
}

/////////////////////////////////////////////////////////////// 
void rotatePickUp() { pwm.setPWM(servoChannelE, 0, 150); }
void rotateDrop() { pwm.setPWM(servoChannelE, 0, 570); }
/////////////////////////////////////////////////////////////// 
void clawOpen() { pwm.setPWM(servoChannelF, 0, 140); }
void clawClosed() { pwm.setPWM(servoChannelF, 0, 300); }
/////////////////////////////////////////////////////////////// 
void toDropHeight() { pwm.setPWM(servoChannelD, 0, 300); }
void toPickUpHeight() { pwm.setPWM(servoChannelD, 0, 200); }
void toCheckHeight() { pwm.setPWM(servoChannelD, 0, 250); }
void toBeginHeight() { pwm.setPWM(servoChannelD, 0, 380); }
/////////////////////////////////////////////////////////////// 
