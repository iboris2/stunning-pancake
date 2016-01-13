#include <Wire.h>

#define PWM0 3
#define PWM1 5


void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
  pinMode(13, OUTPUT);
  pinMode(PWM0, OUTPUT);
  pinMode(PWM1, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);
  delay(50);
  digitalWrite(13, LOW);
  delay(50);
}

void setPwm(unsigned char pwm, unsigned char dir){
  if (dir){
    analogWrite(PWM0,pwm);
    digitalWrite(PWM1,0);
    Serial.print("pwm ");
    Serial.print(pwm);
    Serial.println(", 0");
    
    
  } else {
    analogWrite(PWM1,pwm);
    digitalWrite(PWM0,0);
    Serial.print("pwm 0, ");
    Serial.println(pwm);
  }
}

char reg=0x0;

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  if (howMany) reg = Wire.read();
  //Serial.print("cmd ");
  //Serial.println((int)reg);
  switch(reg){
  case 0x0:
    
    if (Wire.available() != 2) break;
    unsigned char pwm = Wire.read();
    unsigned char dir = Wire.read();
    setPwm(pwm, dir);
    
    
  }
  
  /*
  while (1 < Wire.available()) { // loop through all but the last
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer*/
}
