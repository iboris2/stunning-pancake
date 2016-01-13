// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

void setup() {
  pinMode(13, OUTPUT);
  
  Wire.begin();        // join i2c bus (address optional for master)
}

char i = 50;
char dir = 0;
void loop() {
  /*Wire.requestFrom(8, 6);    // request 6 bytes from slave device #8

  while (Wire.available()) { // slave may send less than requested
    char c = Wire.read(); // receive a byte as character
  }*/
  Wire.beginTransmission(8); // transmit to device #8
  Wire.write(0x0);        // sends five bytes
  Wire.write(i++);
  if (i == 0) dir = ~dir;
  Wire.write(dir);
  Wire.endTransmission();    // stop transmitting
  digitalWrite(13, HIGH);
  delay(50);
  digitalWrite(13, LOW);
  delay(50);
}

