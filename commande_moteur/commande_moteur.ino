#include <Wire.h>
#include <Encoder.h>
#include <TimerOne.h>
#include "CircularBuffer.h"

#include <PID_v1.h>

//#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#define I2C_ADDRESS 8

#define USE_MOTOR_B

// motor A
#define A_EN_PWM 9
#define A_DIR_1 7
#define A_DIR_2 8

#define A_ENCODER_A 3
#define A_ENCODER_B 5

double SetpointA, InputA, OutputA = 0.0;
PID myPIDA(&InputA, &OutputA, &SetpointA,2,5,1, DIRECT);

// motor B
#ifdef USE_MOTOR_B
  #define B_EN_PWM 10
  #define B_DIR_1 11
  #define B_DIR_2 12
  
  #define B_ENCODER_A 2
  #define B_ENCODER_B 4

  double SetpointB, InputB, OutputB = 0.0;
  PID myPIDB(&InputB, &OutputB, &SetpointB,2,5,1, DIRECT);
#endif

Encoder myEncA(A_ENCODER_A, A_ENCODER_B);
#ifdef USE_MOTOR_B
Encoder myEncB(B_ENCODER_A, B_ENCODER_B);
#endif

inline void initMotor(uint8_t pwm_pin, uint8_t dir1_pin, uint8_t dir2_pin) {
  Timer1.pwm(pwm_pin,0);
  pinMode(dir1_pin, OUTPUT);
  pinMode(dir2_pin, OUTPUT);
}

struct Task {
  uint8_t buffer[15];
  uint8_t len;
};

CircularBuffer<Task,10> c_buff;

void setup() {
  Timer1.initialize(40);  // 40 us = 25 kHz
  initMotor(A_EN_PWM, A_DIR_1, A_DIR_2);
#ifdef USE_MOTOR_B
  initMotor(B_EN_PWM, B_DIR_1, B_DIR_2);
#endif

  Wire.begin(I2C_ADDRESS);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event

  Serial.begin(9600);
  Serial.println("go go");
  pinMode(13, OUTPUT);
  myPIDA.SetSampleTime(25);
  myPIDB.SetSampleTime(25);
}
int pos=0;
unsigned int cnt;
void loop() {
  InputA = myEncA.read();
  if (myPIDA.Compute())
    setPwm(OutputA, A_EN_PWM, A_DIR_1, A_DIR_2);
  
  InputB = myEncB.read();
  if (myPIDB.Compute())
    setPwm(OutputB, B_EN_PWM, B_DIR_1, B_DIR_2);
    
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

short int readShort(uint8_t* buff) {
    union u_long {
    byte b[2];
    short int sval;
  } u;
  u.b[0] = *buff++;
  u.b[1] = *buff;
  return u.sval;
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

inline void setPwm(short int pwm, uint8_t pwm_pin, uint8_t dir1_pin, uint8_t dir2_pin) {
  Serial.print("pwm ");
  Serial.println(pwm);
  if (pwm >= 0){
    digitalWrite(dir1_pin, LOW);
    digitalWrite(dir2_pin, HIGH);   
  } else {
    digitalWrite(dir1_pin, HIGH);
    digitalWrite(dir2_pin, LOW);
    pwm = -pwm;
  }
  Timer1.setPwmDuty(pwm_pin,pwm);
}

short int readShort() {
    union u_long {
    byte b[2];
    short int sval;
  } u;
  u.b[0] = Wire.read();
  u.b[1] = Wire.read();
  return u.sval;
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

#define CMD_PWM_AB 0x1
#define CMD_PWM_B  0x2
#define CMD_RST_AB 0x3
#define CMD_RST_B  0x4
//pid
#define CMD_PID_AB 0x5
#define CMD_PID_B  0x6
#define CMD_GOTO_AB 0x7
#define CMD_GOTO_B  0x8
#define CMD_ENABLE_AB 0x9
#define CMD_ENABLE_B  0x10


char reg = 0x0;

void toogle() {
  static uint8_t toogleLed;
  digitalWrite(13, toogleLed);
  toogleLed = ~toogleLed;
}

void receiveEvent(int howMany) {
  long value;
  /* force encoder update to prevent loosing step */
  if (!howMany) return;
  reg = Wire.read();
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
  float kp, ki, kd;
  if (c_buff.remain() <= 0) return;
  Task t = c_buff.pop();
  uint8_t* buff_ptr = t.buffer;
  DEBUG_PRINT("handle task ");
  DEBUG_PRINTLN((int)*buff_ptr);
  int cmd = *buff_ptr++;
  switch(cmd){
  case CMD_PWM_AB:
    if (t.len < 3) break;
    setPwm(readShort(buff_ptr), A_EN_PWM, A_DIR_1, A_DIR_2);
    buff_ptr += 2;
    t.len -= 2;
    // no beak;

#ifdef USE_MOTOR_B
  case CMD_PWM_B:
    if (t.len < 3) break;
    setPwm(readShort(buff_ptr), B_EN_PWM, B_DIR_1, B_DIR_2);
#endif
    break;

  case CMD_RST_AB:
    if (t.len < 5) break;
    myEncA.write(readLong(buff_ptr));
    buff_ptr += 4;
    t.len -= 4;
    // no beak;

#ifdef USE_MOTOR_B
  case CMD_RST_B:
    if (t.len < 5) break;
    myEncB.write(readLong(buff_ptr));
#endif
    break;

  case CMD_PID_AB:
    if (t.len < 13) break;
    kp = readFloat(buff_ptr);
    buff_ptr += 4;
    ki = readFloat(buff_ptr);
    buff_ptr += 4;
    kd = readFloat(buff_ptr);
    myPIDA.SetTunings(kp ,ki, kd);
    break;

#ifdef USE_MOTOR_B
  case CMD_PID_B:
    if (t.len < 13) break;
    kp = readFloat(buff_ptr);
    buff_ptr += 4;
    ki = readFloat(buff_ptr);
    buff_ptr += 4;
    kd = readFloat(buff_ptr);
    myPIDB.SetTunings(kp ,ki, kd);
#endif
    break;

  case CMD_GOTO_AB:
    if (t.len < 5) break;
    SetpointA = readLong(buff_ptr);
    myPIDA.SetMode(AUTOMATIC);
    buff_ptr += 4;
    t.len -= 4;
    // no beak;

#ifdef USE_MOTOR_B
  case CMD_GOTO_B:
    if (t.len < 5) break;
    SetpointB = readLong(buff_ptr);
    myPIDB.SetMode(AUTOMATIC);
#endif
    break;

    case CMD_ENABLE_AB:
    if (t.len < 2) break;
    if (*buff_ptr)
      myPIDA.SetMode(AUTOMATIC);
    else
      myPIDA.SetMode(MANUAL);
    buff_ptr += 1;
    t.len -= 1;
    // no beak;

#ifdef USE_MOTOR_B
  case CMD_ENABLE_B:
    if (t.len < 2) break;
    if (*buff_ptr)
      myPIDB.SetMode(AUTOMATIC);
    else
      myPIDB.SetMode(MANUAL);
#endif
    break;
    
  default:
    break;
  }
}

void requestEvent() {
  toogle();
  long valueA = myEncA.encoder.position;
  Wire.write((uint8_t *)(&valueA), 4);
#ifdef USE_MOTOR_B
  long valueB = myEncB.encoder.position;
  Wire.write((uint8_t *)(&valueB), 4);
#endif
}
