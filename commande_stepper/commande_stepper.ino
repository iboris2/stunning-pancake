#include <AccelStepper.h>
#include <Wire.h>
#include <Encoder.h>
#include "CircularBuffer.h"
#include <avr/io.h>

//#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#define I2C_ADDRESS 9

#define A_STEPPER_DIR_PIN 9     // gris dir
#define A_STEPPER_STEP_PIN 11   // jaune clock
#define A_STEPPER_EN_PIN 10     // marron en

#define A_ENCODER_A 2
#define A_ENCODER_B 4

AccelStepper stepperA(AccelStepper::DRIVER, A_STEPPER_STEP_PIN, A_STEPPER_DIR_PIN);
//Encoder myEncA(A_ENCODER_A, A_ENCODER_B);

#define USE_MOTOR_B

#ifdef USE_MOTOR_B
#define B_STEPPER_DIR_PIN 7     // gris dir
#define B_STEPPER_STEP_PIN 5   // jaune clock
#define B_STEPPER_EN_PIN 6     // marron en

#define B_ENCODER_A 3
#define B_ENCODER_B 8

AccelStepper stepperB(AccelStepper::DRIVER, B_STEPPER_STEP_PIN, B_STEPPER_DIR_PIN);
Encoder myEncB(B_ENCODER_A, B_ENCODER_B);
#endif

#define STEP_PER_REVOLTUTION 400.0
#define ACCELERATION 1600.0

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif
  // 1/4 step : 800 step per revolution
  stepperA.setMaxSpeed(2.7 * STEP_PER_REVOLTUTION);
  stepperA.setAcceleration(ACCELERATION);
  stepperA.setEnablePin(A_STEPPER_EN_PIN);
  stepperA.setPinsInverted(false, false, true);
  stepperA.enableOutputs();
#ifdef USE_MOTOR_B
  stepperB.setMaxSpeed(2.7 * STEP_PER_REVOLTUTION);
  stepperB.setAcceleration(ACCELERATION);
  stepperA.setEnablePin(B_STEPPER_EN_PIN);
  stepperA.setPinsInverted(false, false, true);
  stepperB.enableOutputs();
#endif
  Wire.begin(I2C_ADDRESS);      // join i2c bus with address #8
  TWAR |= 1; /* enable General Call Recognition, used to synchronise steppers */
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
   stepperA.moveTo(10000);
   stepperB.moveTo(10000);
}

struct Task {
  uint8_t buffer[15];
  uint8_t len;
};

CircularBuffer<Task,10> c_buff;

void loop() {
  stepperA.run();
#ifdef USE_MOTOR_B
  stepperB.run();
#endif
  handleTask();
}

float readFloat(uint8_t* buff) {
  union u_float {
    byte b[4];
    float fval;
  } u;
  u.b[0] = *buff++;
  u.b[1] = *buff++;
  u.b[2] = *buff++;
  u.b[3] = *buff;
  return u.fval;
}

long readLong(uint8_t* buff) {
    union u_long {
    byte b[4];
    long lval;
  } u;
  u.b[0] = *buff++;
  u.b[1] = *buff++;
  u.b[2] = *buff++;
  u.b[3] = *buff;
  return u.lval;
}

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
#define CMD_ENABLE_A     0xF
#define CMD_ENABLE_B     0x10

#define CMD_STORE_POS    0xAA
unsigned char reg = 0x0;

volatile long storedEncA = 0;
volatile long storedPosA = 0;

volatile long storedEncB = 0;
volatile long storedPosB = 0;

void receiveEvent(int howMany) {
  long value;
  /* force encoder update to prevent loosing step */
  //myEncA.read();
#ifdef USE_MOTOR_B
  //myEncB.read();
#endif
  if (!howMany) return;
  reg = Wire.read();
  if (reg == CMD_STORE_POS) {/* broadcasted to all i2c slaves */
    //storedEncA = myEncA.read();
    storedPosA = stepperA.currentPosition();
#ifdef USE_MOTOR_B
    //storedEncB = myEncB.read();
    storedPosB = stepperB.currentPosition();
#endif
    return;
  }
  if (reg > CMD_SET_ENC_B) return; /* read register */
  Task* t = c_buff.push();
  if (howMany <= 15)
    t->len = howMany;
  else
    t->len = 15;
  t->buffer[0] = reg;
  for(int i = 1; i < t->len; i++) {
      t->buffer[i] = Wire.read();
  }
}

void handleTask() {
  if (c_buff.remain() <= 0) return;
  Task t = c_buff.pop();
  uint8_t* buff_ptr = t.buffer;
  DEBUG_PRINT("handle task ");
  DEBUG_PRINTLN((int)*buff_ptr);
  int cmd = *buff_ptr++;
  switch(cmd){
  case CMD_MOVE_TO_A:
    if (t.len < 5) break;
    stepperA.moveTo(readLong(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    /* no break */
#ifdef USE_MOTOR_B
  case CMD_MOVE_TO_B:
    if (t.len < 5) break;
    stepperB.moveTo(readLong(buff_ptr));
#endif
    break;

  case CMD_MOVE_A:
    if (t.len < 5) break;
    stepperA.move(readLong(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    /* no break */
#ifdef USE_MOTOR_B
  case CMD_MOVE_B:
    if (t.len < 5) break;
    stepperB.move(readLong(buff_ptr));
#endif
    break;

  case CMD_STOP_A:
    stepperA.stop();
    if (t.len < 2) break;
#ifdef USE_MOTOR_B
  case CMD_STOP_B:
    stepperB.stop();
#endif
    break;

  case CMD_MAX_SPEED_A:
    if (t.len < 5) break;
    stepperA.setMaxSpeed(readFloat(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    /* no break */
#ifdef USE_MOTOR_B
  case CMD_MAX_SPEED_B:
    if (t.len < 5) break;
    stepperB.setMaxSpeed(readFloat(buff_ptr));
#endif
    break;

  case CMD_ACC_A:
    if (t.len < 5) break;
    stepperA.setAcceleration(readFloat(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    /* no break */
#ifdef USE_MOTOR_B
  case CMD_ACC_B:
    if (t.len < 5) break;
    stepperB.setAcceleration(readFloat(buff_ptr));
#endif
    break;

  case CMD_SET_POS_A:
    if (t.len < 5) break;
    stepperA.setCurrentPosition(readLong(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    /* no break */

  case CMD_SET_ENC_A:
    if (t.len < 5) break;
    //myEncA.write(readLong(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    /* no break */
#ifdef USE_MOTOR_B
  case CMD_SET_POS_B:
    if (t.len < 5) break;
    stepperB.setCurrentPosition(readLong(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    /* no break */

  case CMD_SET_ENC_B:
    if (t.len < 5) break;
    //myEncB.write(readLong(buff_ptr));
#endif
    break;

  default:
    break;
  }
}

/* read request */
#define CMD_GET_STATUS      0x21
#define CMD_GET_STORED_DATA 0x22
#define CMD_GET_ENC_A       0x23
#define CMD_GET_POS_A       0x24
#define CMD_GET_ENC_B       0x25
#define CMD_GET_POS_B       0x26

void requestEvent() {
  
  long value;
  /* force encoder update to prevent loosing step */
  //myEncA.read();
#ifdef USE_MOTOR_B
  //myEncB.read();
#endif
  switch(reg){
  case CMD_GET_STATUS:
    value = 0;
    if (stepperA.isRunning())
      value = 1;
#ifdef USE_MOTOR_B
    if (stepperB.isRunning())
      value |= 2;
#endif    
    Wire.write(value);
    DEBUG_PRINT("stat ");DEBUG_PRINTLN((int)value);
    /* no break */
  case CMD_GET_STORED_DATA:
    Wire.write((uint8_t *)(&storedEncA), 4);
    Wire.write((uint8_t *)(&storedPosA), 4);
    DEBUG_PRINT("SencA ");DEBUG_PRINTLN(storedEncA);
    DEBUG_PRINT("SposA ");DEBUG_PRINTLN(storedPosA);
#ifdef USE_MOTOR_B
    Wire.write((uint8_t *)(&storedEncB), 4);
    Wire.write((uint8_t *)(&storedPosB), 4);
#endif
    break;

  case CMD_GET_ENC_A:
    //value = myEncA.read();
    DEBUG_PRINT("encA ");DEBUG_PRINTLN(value);
    Wire.write((uint8_t *)(&value), 4);
    /* no break */

  case CMD_GET_POS_A:
    value = stepperA.currentPosition();
    DEBUG_PRINT("posA ");DEBUG_PRINTLN(value);
    Wire.write((uint8_t *)(&value), 4);
    /* no break */
   
#ifdef USE_MOTOR_B
  case CMD_GET_ENC_B:
    //value = myEncB.read();
    DEBUG_PRINT("encB ");DEBUG_PRINTLN(value);
    Wire.write((uint8_t *)(&value), 4);
    /* no break */

  case CMD_GET_POS_B:
    value = stepperB.currentPosition();
    DEBUG_PRINT("posB ");DEBUG_PRINTLN(value);
    Wire.write((uint8_t *)(&value), 4);
#endif
    break;
  }
}
