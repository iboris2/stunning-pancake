#ifndef STEPPER_H
#define STEPPER_H

#include <stdlib.h>
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <wiring.h>
#endif

class Stepper
{
public:

    Stepper(uint8_t i2c_address);

    void moveTo(long position); 

    void move(long relative);

    void stop();

    void setMaxSpeed(float maxSpeed);

    void setAcceleration(float acceleration);

    void setPosition(long position);

    void setPosition(long position, long encPosition);

    void setEncoderPosition(long encPosition);

    void broadcastStoredPosition();

    bool getStoredPosition(long &encoderPosition, long &position);
    bool getStoredPosition(long &encoderPosition, long &position, long &status);

    bool getPosition(long &encoderPosition, long &position);

    bool getStatus(long &status);

private:
    uint8_t _i2c_address;
};


#endif 
