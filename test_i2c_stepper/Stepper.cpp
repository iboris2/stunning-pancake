#include "Stepper.h"
#include <Wire.h>

/* write request */
#define CMD_MOVE_TO_A    0x1
#define CMD_MOVE_TO_B    0x2
#define CMD_MOVE_A       0x3
#define CMD_MOVE_B       0x4
#define CMD_STOP_A       0x5
#define CMD_STOP_B       0x6
#define CMD_MAX_SPEED_A  0x7
#define CMD_MAX_SPEED_B  0x8
#define CMD_ACC_A        0x9
#define CMD_ACC_B        0xA
#define CMD_SET_POS_A    0xB
#define CMD_SET_ENC_A    0xC
#define CMD_SET_POS_B    0xD
#define CMD_SET_ENC_B    0xE

#define CMD_STORE_POS    0xAA

/* read request */
#define CMD_GET_STATUS      0x21
#define CMD_GET_STORED_DATA 0x22
#define CMD_GET_ENC_A       0x23
#define CMD_GET_POS_A       0x24
#define CMD_GET_ENC_B       0x25
#define CMD_GET_POS_B       0x26

Stepper::Stepper(uint8_t i2c_address) {
    _i2c_address = i2c_address;
}

void Stepper::moveTo(long position) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(CMD_MOVE_TO_A);
    Wire.write((uint8_t* ) &position, 4);
    Wire.endTransmission();
}

void Stepper::move(long relative) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(CMD_MOVE_A);
    Wire.write((uint8_t* ) &relative, 4);
    Wire.endTransmission();
}

void Stepper::stop() {
    Wire.beginTransmission(_i2c_address);
    Wire.write(CMD_STOP_A);
    Wire.endTransmission();
}

void Stepper::setMaxSpeed(float maxSpeed) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(CMD_MAX_SPEED_A);
    Wire.write((uint8_t* ) &maxSpeed, 4);
    Wire.endTransmission();
}

void Stepper::setAcceleration(float acceleration) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(CMD_ACC_A);
    Wire.write((uint8_t* ) &acceleration, 4);
    Wire.endTransmission();
}

void Stepper::setPosition(long position) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(CMD_SET_POS_A);
    Wire.write((uint8_t* ) &position, 4);
    Wire.endTransmission();
}

void Stepper::setPosition(long position, long encPosition) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(CMD_SET_POS_A);
    Wire.write((uint8_t* ) &position, 4);
    Wire.write((uint8_t* ) &encPosition, 4);
    Wire.endTransmission();
}

void Stepper::setEncoderPosition(long encPosition) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(CMD_SET_ENC_A);
    Wire.write((uint8_t* ) &encPosition, 4);
    Wire.endTransmission();
}

void Stepper::broadcastStoredPosition() {
    Wire.beginTransmission(0);
    Wire.write(CMD_STORE_POS);
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

bool Stepper::getStoredPosition(long &encoderPosition, long &position) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(CMD_GET_STORED_DATA);
    Wire.endTransmission();

    Wire.requestFrom(_i2c_address, (uint8_t)8); /* long long */
    if (Wire.available() != 8) return false;
    encoderPosition = readLong();
    position  = readLong();
    return true;
}

bool Stepper::getStoredPosition(long &encoderPosition, long &position, long &status) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(CMD_GET_STATUS);
    Wire.endTransmission();

    Wire.requestFrom(_i2c_address, (uint8_t)9); /* char long long */
    if (Wire.available() != 9) return false;
    status = Wire.read();
    encoderPosition = readLong();
    position  = readLong();
    return true;
}

bool Stepper::getPosition(long &encoderPosition, long &position) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(CMD_GET_ENC_A);
    Wire.endTransmission();

    Wire.requestFrom(_i2c_address, (uint8_t)8); /* long long */
    if (Wire.available() != 8) return false;
    encoderPosition = readLong();
    position  = readLong();
    return true;
}

bool Stepper::getStatus(long &status) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(CMD_GET_STATUS);
    Wire.endTransmission();

    Wire.requestFrom(_i2c_address, (uint8_t)1); /* char */
    if (Wire.available() != 1) return false;
    status = Wire.read();
    return true;
}
