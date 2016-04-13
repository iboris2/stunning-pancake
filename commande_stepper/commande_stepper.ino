#include <AccelStepper.h>
#include <Wire.h>
#include <Encoder.h>
#include "CircularBuffer.h"
#include <avr/io.h>
#include "digitalWriteFast.h"

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

struct CBuffer stepBuffA;

AccelStepper stepperA(&stepBuffA);
//Encoder myEncA(A_ENCODER_A, A_ENCODER_B);

#define USE_MOTOR_B

#ifdef USE_MOTOR_B
#define B_STEPPER_DIR_PIN 7     // gris dir
#define B_STEPPER_STEP_PIN 5   // jaune clock
#define B_STEPPER_EN_PIN 6     // marron en

#define B_ENCODER_A 3
#define B_ENCODER_B 8

struct CBuffer stepBuffB;

AccelStepper stepperB(&stepBuffB);
Encoder myEncB(B_ENCODER_A, B_ENCODER_B);
#endif

#define STEP_PER_REVOLTUTION 1600.0
#define ACCELERATION 400.0

void setup() {
  pinMode(A_STEPPER_DIR_PIN, OUTPUT);
  pinMode(A_STEPPER_STEP_PIN,OUTPUT);
  pinMode(B_STEPPER_DIR_PIN,OUTPUT);
  pinMode(B_STEPPER_STEP_PIN,OUTPUT);

  init_timerOne();

#ifdef DEBUG
  Serial.begin(9600);
#endif
  // 1/4 step : 800 step per revolution
  stepperA.setMaxSpeed(1.0 * STEP_PER_REVOLTUTION);
  stepperA.setAcceleration(ACCELERATION);
  stepperA.setEnablePin(A_STEPPER_EN_PIN);
  stepperA.setPinsInverted(false, false, true);
  //stepperA.enableOutputs();
#ifdef USE_MOTOR_B
  stepperB.setMaxSpeed(1.0 * STEP_PER_REVOLTUTION);
  stepperB.setAcceleration(ACCELERATION);
  stepperB.setEnablePin(B_STEPPER_EN_PIN);
  stepperB.setPinsInverted(false, false, true);
  //stepperB.enableOutputs();
#endif
  Wire.begin(I2C_ADDRESS);      // join i2c bus with address #8
  //TWAR |= 1; /* enable General Call Recognition, used to synchronise steppers */
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event
}

struct Task {
  uint8_t buffer[15];
  uint8_t len;
};

void init_timerOne()
{
  TCCR1A = 0; // _BV(COM1A0)  | _BV(COM1B0) ;
  TCCR1B = _BV(CS11); // 2mhz

  TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);
  TIFR1  = _BV(OCF1A) | _BV(OCF1B);
}

ISR(TIMER1_COMPB_vect)
{
  static uint8_t step = false;
  static uint8_t dir = 0;
  if (dir) {
    digitalWriteFast(B_STEPPER_DIR_PIN, 1);
  } else {
    digitalWriteFast(B_STEPPER_DIR_PIN, 0);
  }
  if (step) {
    digitalWriteFast(B_STEPPER_STEP_PIN, 1);
  }
  struct Command cmd;
  step = pull(&stepBuffB, &cmd);
  if (cmd.dir_buff == 255) step = 0;
  if (step) {
    OCR1B += cmd.step_buff;
    dir = cmd.dir_buff;
  }
  digitalWriteFast(B_STEPPER_STEP_PIN, 0);
}

ISR(TIMER1_COMPA_vect)
{
  static uint8_t step = false;
  static uint8_t dir = 0;
  if (dir) {
    digitalWriteFast(A_STEPPER_DIR_PIN, 1);
  } else {
    digitalWriteFast(A_STEPPER_DIR_PIN, 0);
  }
  if (step) {
    digitalWriteFast(A_STEPPER_STEP_PIN, 1);
  }
  struct Command cmd;
  step = pull(&stepBuffA, &cmd);
  if (cmd.dir_buff == 255) step = 0;
  if (step) {
    OCR1A += cmd.step_buff;
    dir = cmd.dir_buff;
  }
  digitalWriteFast(A_STEPPER_STEP_PIN, 0);
}


CircularBuffer<Task,10> c_buff;

void loop() {
  stepperA.runStepBuffer();
#ifdef USE_MOTOR_B
  stepperB.runStepBuffer();
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
#define CMD_SET_POS_B    0xC
#define CMD_ENABLE_A     0xD
#define CMD_ENABLE_B     0xE

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
  if (reg > CMD_ENABLE_B) return; /* read register */
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
    stepperA.enableOutputs();
    stepperA.moveTo(readLong(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    /* no break */
#ifdef USE_MOTOR_B
  case CMD_MOVE_TO_B:
    if (t.len < 5) break;
    stepperB.enableOutputs();
    stepperB.moveTo(readLong(buff_ptr));
#endif
    break;

  case CMD_MOVE_A:
    if (t.len < 5) break;
    stepperA.enableOutputs();
    stepperA.move(readLong(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    /* no break */
#ifdef USE_MOTOR_B
  case CMD_MOVE_B:
    if (t.len < 5) break;
    stepperB.enableOutputs();
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
    DEBUG_PRINT("speedA ");DEBUG_PRINTLN(readFloat(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    /* no break */
#ifdef USE_MOTOR_B
  case CMD_MAX_SPEED_B:
    if (t.len < 5) break;
    stepperB.setMaxSpeed(readFloat(buff_ptr));
    DEBUG_PRINT("speedB ");DEBUG_PRINTLN(readFloat(buff_ptr));
#endif
    break;

  case CMD_ACC_A:
    if (t.len < 5) break;
    stepperA.setAcceleration(readFloat(buff_ptr));
    DEBUG_PRINT("accA ");DEBUG_PRINTLN(readFloat(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    /* no break */
#ifdef USE_MOTOR_B
  case CMD_ACC_B:
    if (t.len < 5) break;
    stepperB.setAcceleration(readFloat(buff_ptr));
    DEBUG_PRINT("accB ");DEBUG_PRINTLN(readFloat(buff_ptr));
#endif
    break;

  case CMD_SET_POS_A:
    if (t.len < 5) break;
    stepperA.setCurrentPosition(readLong(buff_ptr));
    DEBUG_PRINT("Set pos A ");DEBUG_PRINTLN(readLong(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    /* no break */

#ifdef USE_MOTOR_B
  case CMD_SET_POS_B:
    if (t.len < 5) break;
    stepperB.setCurrentPosition(readLong(buff_ptr));
    DEBUG_PRINT("Set pos B ");DEBUG_PRINTLN(readLong(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
#endif
    break;
    
 case CMD_ENABLE_A:
    if (t.len < 2) break;
    if (*buff_ptr)
      stepperA.enableOutputs();
    else
      stepperA.disableOutputs();
    buff_ptr += 1;
    t.len -= 1; 
#ifdef USE_MOTOR_B
  case CMD_ENABLE_B:
    if (t.len < 2) break;
    if (*buff_ptr)
      stepperB.enableOutputs();
    else
      stepperB.disableOutputs();
#endif
    break;
  default:
    break;
  }
}

/* read request */
#define CMD_REMAINING_A     0x21
#define CMD_REMAINING_B     0x22
#define CMD_GET_POS_A       0x23
#define CMD_GET_POS_B       0x24

void requestEvent() {
  
  long value;
  long values[2];
  long offset=0;
  switch(reg){
  case CMD_REMAINING_A:
    values[offset++] = stepperA.distanceToGo();
    DEBUG_PRINT("remain A ");DEBUG_PRINTLN(values[offset-1]);
    /* no break */
  #ifdef USE_MOTOR_B
    values[offset++] = stepperB.distanceToGo();
    DEBUG_PRINT("remain B ");DEBUG_PRINTLN(values[offset-1]);
  #endif    
    Wire.write((uint8_t *)(values), 4*offset);
    break;

  case CMD_GET_POS_A:
    values[offset++] = stepperA.currentPosition();
    DEBUG_PRINT("posA ");DEBUG_PRINTLN(values[offset-1]);
    /* no break */
#ifdef USE_MOTOR_B
  case CMD_GET_POS_B:
    values[offset++] = stepperB.currentPosition();
    DEBUG_PRINT("posB ");DEBUG_PRINTLN(values[offset-1]);

#endif
    Wire.write((uint8_t *)(values), 4*offset);
    break;
  }
}
