#include <Wire.h>
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
#include <TimerOne.h>

#define PWM0 9
#define PWM1 10

#define ENCODER_A 2
#define ENCODER_B 3

Encoder myEnc(ENCODER_A, ENCODER_B);


void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event

  Timer1.initialize(40);  // 40 us = 25 kHz
  Timer1.pwm(PWM0,0);
  Timer1.pwm(PWM1,0);

  Serial.begin(9600);
  Serial.println("go go");
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}

void setPwm(short int pwm){
  Serial.print("pwm ");
  Serial.println(pwm);
  if (pwm >= 0){
    Timer1.setPwmDuty(PWM1,0);
    Timer1.setPwmDuty(PWM0,pwm);    
  } else {
    Timer1.setPwmDuty(PWM0,0);
    Timer1.setPwmDuty(PWM1,-pwm);
  }
}

void reset(long value){
  myEnc.write(value);
  Serial.print("reset ");
  Serial.println(value);
}

char reg=0x0;

void receiveEvent(int howMany) {
  if (howMany) reg = Wire.read();
  switch(reg){
  case 0x1:{
    if (howMany != 3) break;
    short int pwm = (short int)Wire.read() | (short int)Wire.read() << 8;
    setPwm(pwm);
    break;
  }

  case 0x2:{
    long value = 0;
    int shift = 0;
    if (howMany != 5) break;
    value = (long)Wire.read() | (long)Wire.read() << 8 | (long)Wire.read() << 16 | (long)Wire.read() << 24;
    reset(value);
    break;
  }

  default:
    break;
  }
  // flush read buffer
  while (Wire.available()) Wire.read();
  
}

void requestEvent() {
  long value = myEnc.encoder.position;
  Serial.print("pos ");Serial.println(value);
  Wire.write((uint8_t *)(&value), 4);
}
