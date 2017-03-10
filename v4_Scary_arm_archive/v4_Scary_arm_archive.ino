#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

int servoPosA = 150; // Current servo A position (pulse length)
int servoPosB = 150; // Current servo B position (pulse length)
int servoPosC = 150; // Current servo C position (pulse length)
const int servoDelayA = 3; // Servo A delay variables, in milliseconds
const int servoDelayB = 3; // Servo B delay variables, in milliseconds
const int servoDelayC = 3; // Servo C delay variables, in milliseconds

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

void loop() {
  if(Serial.available()) {
    controlMode = Serial.read();
    Serial.println(controlMode);
  }
  switch(controlMode) {
    case 's':         // "Stop" 
      returnToControlMode = 's';
      break;
    case 'g':         // Auto Run ("Go")
      autoRun();
      controlMode = 's';
      break;
    ///////////////////////////////////////////////////////////////  
    case 'a':         // Joint A CW/CCW (TODO: Determine Direction)
      moveServos(servoPosA + 1, servoPosB, servoPosC);
      controlMode = returnToControlMode;
      break;
    case 'b':         // Joint A CW/CCW (TODO: Determine Direction)
      moveServos(servoPosA - 1, servoPosB, servoPosC);
      controlMode = returnToControlMode;
      break;
    /////////////////////////////////////////////////////////////// 
    case 'c':         // Joint B CW/CCW (TODO: Determine Direction)
      moveServos(servoPosA, servoPosB + 1, servoPosC);
      controlMode = returnToControlMode;
      break;
    case 'd':         // Joint B CW/CCW (TODO: Determine Direction)
      moveServos(servoPosA, servoPosB -1, servoPosC);
      controlMode = returnToControlMode;
      break;
    /////////////////////////////////////////////////////////////// 
    case 'e':         // Joint C CW/CCW (TODO: Determine Direction)
      moveServos(servoPosA, servoPosB, servoPosC + 1);
      controlMode = returnToControlMode;
      break;
    case 'f':         // Joint C CW/CCW (TODO: Determine Direction)
      moveServos(servoPosA, servoPosB, servoPosC - 1);
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
      potValA = map(analogRead(potPinA), 470, 120, 100, 650);
      potValB = map(analogRead(potPinB), 470, 120, 100, 650);
      potValC = map(analogRead(potPinC), 120, 470, 100, 650);
      moveServos(potValA, potValB, potValC);
      //Serial.print("Pots: ");
      //Serial.print(potValA);
      //Serial.print(", ");
      //Serial.print(potValB);
      //Serial.print(", ");
      //Serial.println(potValC);
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
  } 
}

void lift(int targetDeg) {
  int dir;
  if(currentLiftPosition < targetDeg) {
    digitalWrite(stepperDirectionC, 0);
  }
  else if(currentLiftPosition > targetDeg) {
    digitalWrite(stepperDirectionC, 1);
  }
  
  int stepStat = 0;
  for(int i = map(targetDeg, 0, 90, 0, 2500); i > 0; i--) {
    digitalWrite(stepperStepC, stepStat);
    delayMicroseconds(750);
    if(stepStat == 0) {
      stepStat = 1;
    } else {
      stepStat = 0;
    }
  }
}

void moveServos(int posA, int posB, int posC) {
  int servoDirA;
  int servoDirB;
  int servoDirC; 
  /////////////////////////////////////////////////////////////// 
  if(servoPosA < posA) {
    servoDirA = 1;
  } else {
    servoDirA = -1;
  }
  /////////////////////////////////////////////////////////////// 
  if(servoPosB < posB) {
    servoDirB = 1;
  } else {
    servoDirB = -1;
  }
  /////////////////////////////////////////////////////////////// 
  if(servoPosC < posC) {
    servoDirC = 1;
  } else {
    servoDirC = -1;
  }  
  /////////////////////////////////////////////////////////////// 
  while(servoPosA != posA) {
    servoPosA+=servoDirA;
    pwm.setPWM(servoChannelA, 0, servoPosA);
    delay(servoDelayA);
  }
  /////////////////////////////////////////////////////////////// 
  while(servoPosB != posB) {
    servoPosB+=servoDirB;
    pwm.setPWM(servoChannelB, 0, servoPosB);
    delay(servoDelayB);
  }
  /////////////////////////////////////////////////////////////// 
  while(servoPosC != posC) {
    servoPosC+=servoDirC;
    pwm.setPWM(servoChannelC, 0, servoPosC);
    delay(servoDelayC);
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
