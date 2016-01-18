#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>
#include <Encoder.h>
#include "CircularBuffer.h"

#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#define I2C_ADDRESS 9

// The X Stepper pins
#define A_STEPPER_DIR_PIN 7
#define A_STEPPER1_STEP_PIN 8
// The Y stepper pins
#define B_STEPPER_DIR_PIN 9
#define B_STEPPER_STEP_PIN 10

#define A_ENCODER_A 2
#define A_ENCODER_B 4
  
#define B_ENCODER_A 3
#define B_ENCODER_B 5

AccelStepper stepperA(AccelStepper::DRIVER, A_STEPPER_DIR_PIN, A_STEPPER_DIR_PIN);
AccelStepper stepperB(AccelStepper::DRIVER, B_STEPPER_STEP_PIN, B_STEPPER_DIR_PIN);

Encoder myEncA(A_ENCODER_A, A_ENCODER_B);
Encoder myEncB(B_ENCODER_A, B_ENCODER_B);

#define STEP_PER_REVOLTUTION 800.0
#define ACCELERATION 800.0

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif
  // 1/4 step : 800 step per revolution
  stepperA.setMaxSpeed(2.5 * STEP_PER_REVOLTUTION);
  stepperA.setAcceleration(ACCELERATION);

  stepperB.setMaxSpeed(2.5 * STEP_PER_REVOLTUTION);
  stepperB.setAcceleration(ACCELERATION);

  Wire.begin(I2C_ADDRESS);      // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
}

struct Task {
  uint8_t buffer[15];
  uint8_t len;
};

CircularBuffer<Task,10> c_buff;

void loop() {
    stepperA.run();
    stepperB.run();
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
char reg = 0x0;

void receiveEvent(int howMany) {
  long value;
  // force encoder update to prevent loosing step
  myEncA.read();
  myEncB.read();
  if (!howMany) return;
  Task* t = c_buff.push();
  if (howMany <= 16)
    t->len = howMany;
  else
    t->len = 16;
  for(int i = 0; i < t->len; i++) {
      t->buffer[i] = Wire.read();
  }
  reg = t->buffer[0];
}

void handleTask() {
  if (c_buff.remain() <= 0) return;
  Task t = c_buff.pop();
  uint8_t* buff_ptr = t.buffer;
  DEBUG_PRINT("handle task ");
  DEBUG_PRINTLN((int)*buff_ptr);
  switch(*buff_ptr++){
  case CMD_MOVE_TO_AB:
    if (t.len < 5) break;
    stepperA.moveTo(readLong(buff_ptr));
    buff_ptr += 4; t.len -= 4;
    // no beak;

  case CMD_MOVE_TO_B:
    if (t.len < 5) break;
    stepperB.moveTo(readLong(buff_ptr));
    break;

  case CMD_MOVE_AB:
    if (t.len < 5) break;
    stepperA.move(readLong(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    // no beak;

  case CMD_MOVE_B:
    if (t.len < 5) break;
    stepperB.move(readLong(buff_ptr));
    break;

  case CMD_STOP_AB:
    stepperA.stop();
    if (t.len < 2) break;

  case CMD_STOP_B:
    stepperB.stop();
    break;

  case CMD_MAX_SPEED_AB:
    if (t.len < 5) break;
    stepperA.setMaxSpeed(readFloat(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    // no beak;

  case CMD_MAX_SPEED_B:
    if (t.len < 5) break;
    stepperB.setMaxSpeed(readFloat(buff_ptr));
    break;

  case CMD_ACC_AB:
    if (t.len < 5) break;
    stepperA.setAcceleration(readFloat(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    // no beak;

  case CMD_ACC_B:
    if (t.len < 5) break;
    stepperB.setAcceleration(readFloat(buff_ptr));
    break;
    
  case CMD_SET_POS_AB:
    if (t.len < 5) break;
    stepperA.setCurrentPosition(readLong(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    // no beak;

  case CMD_SET_POS_B:
    if (t.len < 5) break;
    stepperB.setCurrentPosition(readLong(buff_ptr));
    break;

  default:
    break;
  }
}

//read request
#define CMD_MOT_POS    0x11

void requestEvent() {
  
  long valueA, valueB;
  switch(reg){
    case CMD_MOT_POS:
    default:
    valueA = stepperA.currentPosition();
    valueB = stepperB.currentPosition();
    DEBUG_PRINT("posA ");DEBUG_PRINTLN(valueA);
    DEBUG_PRINT("posB ");DEBUG_PRINTLN(valueB);
    Wire.write((uint8_t *)(&valueA), 4);
    Wire.write((uint8_t *)(&valueB), 4);

    valueA = myEncA.read();
    valueB = myEncB.read();
    DEBUG_PRINT("encA ");DEBUG_PRINTLN(valueA);
    DEBUG_PRINT("encB ");DEBUG_PRINTLN(valueB);
    Wire.write((uint8_t *)(&valueA), 4);
    Wire.write((uint8_t *)(&valueB), 4);
    break;
  }
}
