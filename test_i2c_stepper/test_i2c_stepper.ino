#include <TimerOne.h>

#include <Wire.h>

#define I2C_ADDRESS 9

void setup() {
  Timer1.initialize(80);
  Timer1.pwm(9, 512);
  pinMode(13, OUTPUT);
  Wire.begin();        // join i2c bus (address optional for master)
}

#define CMD_MOVE_TO_AB   0x1
#define CMD_MOVE_TO_B    0x2
#define CMD_MOVE_AB      0x3
#define CMD_MOVE_B       0x4
#define CMD_STOP_AB      0x5
#define CMD_STOP_B       0x6
#define CMD_MAX_SPEED_AB 0x7
#define CMD_MAX_SPEED_B  0x8
#define CMD_ACC_AB       0x9
#define CMD_ACC_B        0xA
#define CMD_SET_POS_AB   0xB
#define CMD_SET_POS_B    0xC
//read request
#define CMD_MOT_POS    0x11

void moveTo(long postionA, long postionB);
void move(long relativeA, long relativeB);
void stop();
void setMaxSpeed(float maxSpeedA, float maxSpeedB);
void setAcceleration(float accelerationA, float accelerationB);
void setCurrentPosition(long postionA, long postionB);

int getPosition(long *postionA, long *postionB, long *encoderA, long *encoderB);


void moveTo(long postionA, long postionB) {
  Wire.beginTransmission(I2C_ADDRESS); // transmit to device #8
  Wire.write(CMD_MOVE_TO_AB);        // sends five bytes
  Wire.write((uint8_t* ) &postionA,4);
  Wire.write((uint8_t* ) &postionB,4);
  Wire.endTransmission();
}

void move(long relativeA, long relativeB) {
  Wire.beginTransmission(I2C_ADDRESS); // transmit to device #8
  Wire.write(CMD_MOVE_AB);        // sends five bytes
  Wire.write((uint8_t* ) &relativeA,4);
  Wire.write((uint8_t* ) &relativeB,4);
  Wire.endTransmission();
}

void stop() {
  Wire.beginTransmission(I2C_ADDRESS); // transmit to device #8
  Wire.write(CMD_STOP_AB);        // sends five bytes
  Wire.write('s');
  Wire.endTransmission();
}

void setMaxSpeed(float maxSpeedA, float maxSpeedB) {
  Wire.beginTransmission(I2C_ADDRESS); // transmit to device #8
  Wire.write(CMD_MAX_SPEED_AB);        // sends five bytes
  Wire.write((uint8_t* ) &maxSpeedA,4);
  Wire.write((uint8_t* ) &maxSpeedB,4);
  Wire.endTransmission();
}

void setAcceleration(float accelerationA, float accelerationB) {
  Wire.beginTransmission(I2C_ADDRESS); // transmit to device #8
  Wire.write(CMD_ACC_AB);        // sends five bytes
  Wire.write((uint8_t* ) &accelerationA,4);
  Wire.write((uint8_t* ) &accelerationB,4);
  Wire.endTransmission();
}

void setCurrentPosition(long postionA, long postionB) {
  Wire.beginTransmission(I2C_ADDRESS); // transmit to device #8
  Wire.write(CMD_SET_POS_AB);        // sends five bytes
  Wire.write((uint8_t* ) &postionA,4);
  Wire.write((uint8_t* ) &postionB,4);
  Wire.endTransmission();
}

long readLong() {
    union u_long {
    byte b[4];
    long lval;
  } u;
  u.b[0] = Wire.read();
  u.b[1] = Wire.read();
  u.b[2] = Wire.read();
  u.b[3] = Wire.read();
  return u.lval;
}

int getPosition(long *postionA, long *postionB, long *encoderA, long *encoderB) {
  Wire.requestFrom(I2C_ADDRESS, 8);
  if (Wire.available() != 8) return -1;
  *postionA = readLong();
  *postionB = readLong();
  *encoderA = readLong();
  *encoderB = readLong();
}

void blink() {
  digitalWrite(13, HIGH);
  delay(50);
  digitalWrite(13, LOW);
  delay(50);
}

char dir = 0;
long value = 0;/*
void loop() {
long postionA, postionB, encoderA, encoderB;

blink();
moveTo(6000, 6000);
blink();
delay(6000);
move(8000, 8000);
blink();
delay(6000);
stop();
blink();

getPosition(&postionA, &postionB, &encoderA, &encoderB);
blink();
}*/
/*
float v = 10.0;
void loop() {
  setAcceleration(v, v);
  v += 1.0;
}*/

void loop() {
  long postionA, postionB, encoderA, encoderB;
  getPosition(&postionA, &postionB, &encoderA, &encoderB);
}

