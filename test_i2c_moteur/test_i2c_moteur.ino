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

#define I2C_SLAVE 10

void setup() {
  pinMode(13, OUTPUT);
  //pinMode(A4, INPUT_PULLUP);
 // pinMode(A5, INPUT_PULLUP);
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);
}

void pwm(short int value){
  Wire.beginTransmission(I2C_SLAVE); // transmit to device #8
  Wire.write(PWM_CMD);        // sends five bytes
  Wire.write((uint8_t* ) &value,2);
  Wire.endTransmission();
}

void reset(long value){
  Wire.beginTransmission(I2C_SLAVE); // transmit to device #8
  Wire.write(RESET_CMD);        // sends five bytes
  Wire.write((uint8_t* ) &value,4);
  Wire.endTransmission();
}

long read(){
  static long storedValue = 0;
  Wire.requestFrom(I2C_SLAVE, 4);    // request 4 bytes from slave device #8
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

  if (Serial.available()) {
    switch((char)Serial.read()){
      case 'u': pwm(512);delay(15);pwm(1023); break;
      case 'd': pwm(-512);delay(15);pwm(-1023);break;
      case 's': pwm(0);break;
    }
    digitalWrite(13, value);
    value = ~value;
    Serial.write('1');
  }
}

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

/*void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}*/

