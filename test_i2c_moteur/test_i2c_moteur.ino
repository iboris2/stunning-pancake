// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>
#define PWM_CMD 0x1
#define RESET_CMD 0x2

void setup() {
  pinMode(13, OUTPUT);
  Wire.begin();        // join i2c bus (address optional for master)
}

void pwm(short int value){
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(PWM_CMD);        // sends five bytes
  Wire.write((uint8_t* ) &value,2);
  Wire.endTransmission();
}

void reset(long value){
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(RESET_CMD);        // sends five bytes
  Wire.write((uint8_t* ) &value,4);
  Wire.endTransmission();
}

long read(){
  static long storedValue = 0;
  Wire.requestFrom(8, 4);    // request 4 bytes from slave device #8
  if (Wire.available() != 4) {
    while(Wire.available()) {
      Wire.read(); // flush
    }
  } else {
    storedValue = (long)Wire.read() | (long)Wire.read() << 8 | (long)Wire.read() << 16 | (long)Wire.read() << 24; 
  }
  return storedValue;
}

short int i = 0;
char dir = 0;
long value = 0;
void loop() {
  pwm(i+=64);
  if (i > 1024) i = -1024;
  digitalWrite(13, HIGH);
  delay(50);
  digitalWrite(13, LOW);
  delay(50);

  if (i==0){
    value = read();
    reset(value+1);
  }

}

