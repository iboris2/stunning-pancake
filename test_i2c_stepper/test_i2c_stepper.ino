#include <TimerOne.h>
#include "Stepper.h"
#include <Wire.h>

#define I2C_ADDRESS 9

void setup() {
  //Timer1.initialize(80);
  //Timer1.pwm(9, 512);
  pinMode(13, OUTPUT);
  Wire.begin();        // join i2c bus (address optional for master)
}


void blink() {
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
}

Stepper stepper(I2C_ADDRESS);

void loop() {
  long encoder, position, status;

  blink();
  stepper.moveTo(6000);
  blink();
  delay(1000);
  stepper.move(8000);
  blink();
  delay(1000);
  stepper.stop();
  blink();
  stepper.broadcastStoredPosition();
  blink();
  stepper.getStoredPosition(encoder, position,status);
  blink();
  stepper.getStoredPosition(encoder, position);
  blink();
  stepper.getPosition(encoder, position);
  blink();
}

/*
float v = 10.0;
void loop() {
  setAcceleration(v, v);
  v += 1.0;
}*/

/*
void loop() {
  long postionA, postionB, encoderA, encoderB;
  getPosition(&postionA, &postionB, &encoderA, &encoderB);
}*/

