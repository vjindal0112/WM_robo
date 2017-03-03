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


int servo0Pos;
int servo1Pos;
int servo2Pos;
int MOVEDELAY = 12;
char receivedChar;


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
    delay(MOVEDELAY);
  }
  while(servo1Pos != pos1) {
    servo1Pos+=dir1;
    pwm.setPWM(1, 0, servo1Pos);
    delay(MOVEDELAY);
  }
  while(servo2Pos != pos2) {
    servo2Pos+=dir2;
    pwm.setPWM(2, 0, servo2Pos);
    delay(MOVEDELAY);
  }
}

// set positions
void rotatePickUp() {
  pwm.setPWM(4, 0, 150); //rotate (570 up, 150 down)
}

void rotateDrop() {
  pwm.setPWM(4, 0, 570); //rotate (570 up, 150 down)
}

void clawOpen() {
  pwm.setPWM(3, 0, 300); //claw (530 closed, 300 open)
}

void clawClosed() {
  pwm.setPWM(3, 0, 530); //claw (530 closed, 300 open)
}

void toDropHeight() {
}

void toPickUpHeight() {
  
}

void toCheckHeight() {
  // check if pennies right under or not i.e. fine turning height
  
}

void fineTune() {
  while(receivedChar != 'g') {
    receivedChar = Serial.read();
    if(receivedChar == 'w') {
      moveServos(servo0pos + 1, servo1pos, servo2pos);
    }
    if(receivedChar == 's') {
      moveServos(servo0pos - 1, servo1pos, servo2pos);
    }
    if(receivedChar == 'r') {
      moveServos(servo0pos, servo1pos + 1, servo2pos);
    }
    if(receivedChar == 'f') {
      moveServos(servo0pos, servo1pos - 1, servo2pos);
    }
    if(receivedChar == 'y') {
      moveServos(servo0pos, servo1pos, servo2pos + 1);
    }
    if(receivedChar == 'h') {
      moveServos(servo0pos, servo1pos, servo2pos - 1);
    }
  }
}

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0) {
    
  }  
}
