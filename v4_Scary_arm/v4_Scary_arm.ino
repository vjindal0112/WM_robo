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


uint8_t servonum = 0;
int potpin = 0;  // analog pin used to connect the potentiometer
int buttonState = 0;  
int servo0Pos;
int servo1Pos;
int servo2Pos;
int MOVEDELAY = 20;
int DISPENSEDELAY = 3;


void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  yield();
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
    pwm.setPWM(0, 0, servo0Pos);
    delay(10);
  }
  while(servo1Pos != pos1) {
    servo1Pos+=dir1;
    pwm.setPWM(1, 0, servo1Pos);
    delay(10);
  }
  while(servo2Pos != pos2) {
    servo2Pos+=dir2;
    pwm.setPWM(2, 0, servo2Pos);
    delay(10);
  }
}

void bringDown() {
  for(int i = 250; i < 585; i++) {
    pwm.setPWM(10, 0, i);
    pwm.setPWM(11, 0, i);
    delay(17);
  }
}

void rotateDown() {
  pwm.setPWM(4, 0, 150); //rotate (570 up, 150 down)
}

void rotateUp() {
  pwm.setPWM(4, 0, 570); //rotate (570 up, 150 down)
}

void clawOpen() {
  pwm.setPWM(3, 0, 300); //claw (530 closed, 300 open)
}

void clawClosed() {
  pwm.setPWM(3, 0, 530); //claw (530 closed, 300 open)

}

void loop() {
  // put your main code here, to run repeatedly:
  bringDown();
  rotateUp();
  clawClosed();
  moveServos(590, 633, false);
  moveServos(505, 463, true);
  delay(200);
  clawOpen();
  delay(400);
  rotateDown();
  delay(1000);
  moveServos(228, 410, true);
  delay(2000);
  clawClosed();
  delay(200);
  rotateUp();
  delay(500);
  moveServos(246, 473, false);
  clawOpen();
  delay(500);
  moveServos(228, 410, true);
  rotateDown();
  delay(200);
  moveServos(345, 479, true);
  delay(200);
  moveServos(344, 502, true);
  delay(200);
  clawClosed();
  moveServos(228, 410, true);
  rotateUp();
  moveServos(230, 490, true);
  clawOpen();
  delay(500);
  
   
  
}
