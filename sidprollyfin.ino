#include <Servo.h>

Servo rightLeg;
Servo leftLeg;
Servo rightArm;
Servo leftArm;
Servo head;

int int1 = 2;
int int2 = 3;
int int3 = 4;
int int4 = 5;
int int5 = 6;
int int6 = 7;
int int7 = 8;
int int8 = 9;

int low = 0;
int high = 160;

int headValue = constrain(headValue, 50, 130);

char openMV;
char OAKD;

long last1 = micros();

void setup(){
  Serial.begin(9600);
  rightLeg.attach(12);
  leftLeg.attach(11);
  rightArm.attach(21);
  leftArm.attach(53);
  head.attach(15);

  headValue = 90;
  head.write(headValue);

  pinMode(int1, OUTPUT);
  pinMode(int2, OUTPUT);
  pinMode(int3, OUTPUT);
  pinMode(int4, OUTPUT);
  pinMode(int5, OUTPUT);
  pinMode(int6, OUTPUT);
  pinMode(int7, OUTPUT);
  pinMode(int8, OUTPUT);
}

void loop(){
  if(openMV == 'a'){
    return;
  }

  if(Serial.available()){
    OAKD = Serial.read();
  }

  switch(OAKD){
    // OAKD
    case 'e': // cara a la derecha
      turnRight();
      servoStop();
      break;
    case 'm': // cara a la izquierda
      turnLeft();
      servoStop;
      break;
    case 'b': // cara al centro
      t1200();
      servoMovement();
      break;
    case 'n': // cerca de la cara
      wait();
      sayHi();
    case 'u':
      lookUp();
      break;
    case 'g':
      lookDown();
      break;
    case 'z':
      lookStraight();
      break;

    default:
      wait();
      servoStop();
      break;
  }
}

bool moved = false;
void servoMovement(){
  if(micros()-last1 > 1000000 && !moved){
    moved = true;
    last1 = micros();
    leftLeg.write(80);
    rightLeg.write(90);
    leftArm.write(130);
    rightArm.write(130);
  }
  if(micros()-last1 > 1000000 && moved){
    last1 = micros();
    moved = false;
    leftLeg.write(0); //leg bends
    rightLeg.write(10); //leg straight
    leftArm.write(0); //arm straights
    rightArm.write(0); //arm bends
  }
  
}

void servoStop(){
  leftLeg.write(80);
  rightLeg.write(0);
  leftArm.write(0); //arm bends
  rightArm.write(130); //arm straights 
}

void sayHi(){
  if(micros()-last1 > 1000000 && !moved){
    moved = true;
    last1 = micros();
    leftLeg.write(80);
    rightLeg.write(0);
    leftArm.write(0);
    rightArm.write(0); 
  }
  if(micros()-last1 > 1000000 && moved){
    moved = false;
    last1 = micros();
    leftLeg.write(80);
    rightLeg.write(0);
    leftArm.write(0);
    rightArm.write(130);
  }
}

void lookUp(){
  head.write(headValue - 30);
}

void lookStraight(){
  head.write(headValue);
}

void lookDown(){
  head.write(headValue + 30);
}

void t1200(){
  analogWrite(int1, low);
  analogWrite(int2, low);
  analogWrite(int3, low);
  analogWrite(int4, low);
  analogWrite(int5, high);
  analogWrite(int6, low);
  analogWrite(int7, high);
  analogWrite(int8, low);

}

void turnLeft(){
  analogWrite(int1, low);
  analogWrite(int2, high);
  analogWrite(int3, high);
  analogWrite(int4, low);
  analogWrite(int5, low);
  analogWrite(int6, high);
  analogWrite(int7, high);
  analogWrite(int8, low);

}

void turnRight(){
  analogWrite(int1, high);
  analogWrite(int2, low);
  analogWrite(int3, low);
  analogWrite(int4, high);
  analogWrite(int5, high);
  analogWrite(int6, low);
  analogWrite(int7, low);
  analogWrite(int8, high);

}

void wait(){
  analogWrite(int1, low);
  analogWrite(int2, low);
  analogWrite(int3, low);
  analogWrite(int4, low);
  analogWrite(int5, low);
  analogWrite(int6, low);
  analogWrite(int7, low);
  analogWrite(int8, low);

}
