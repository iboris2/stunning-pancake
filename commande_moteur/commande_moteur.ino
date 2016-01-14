#include <Wire.h>
#include <Encoder.h>

#define PWM0 3
#define PWM1 5

#define ENCODER_A 2
#define ENCODER_B 3

Encoder myEnc(ENCODER_A, ENCODER_B);


void setup() {
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event

  Serial.begin(9600);
  pinMode(13, OUTPUT);
  pinMode(PWM0, OUTPUT);
  pinMode(PWM1, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
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

void receiveEvent(int howMany) {
  if (howMany) reg = Wire.read();
  
  switch(reg){
  case 0x1:
    if (Wire.available() != 2) break;
    unsigned char pwm = Wire.read();
    unsigned char dir = Wire.read();
    setPwm(pwm, dir);
    break;

  case 0x2:
    long value = 0;
    int shift = 0;
    while ()
    if (Wire.available()) value = Wire.read() << 24;
)
  default:
    break;
  }
  // flush read buffer
  while (Wire.available()) Wire.read();
  
}

void requestEvent() {
  long value = myEnc.read();
  Wire.write((uint8_t *)(&value), 4);
}
