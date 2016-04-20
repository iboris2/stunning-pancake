#include <Wire.h>
#include <Encoder.h>
#include <TimerOne.h>
#include "CircularBuffer.h"

#include <PID_v1.h>

//#define DEBUG


#define I2C_ADDRESS 12
//#define I2C_ADDRESS 14


#define SAMPLETIME 20
#define DEADBAND 80
#define BRIDE_PWM 256

#ifdef DEBUG
  #undef SAMPLETIME
  #define SAMPLETIME 25
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#define USE_MOTOR_B

// motor A
#define A_EN_PWM 9
#define A_DIR_1 7
#define A_DIR_2 8

#define A_ENCODER_A 3
#define A_ENCODER_B 5

double SetpointA , InputA, OutputA = 0.0;
short int lastPwmA = 0;
short int maxPwmA = 1024;

PID myPIDA(&InputA, &OutputA, &SetpointA,2,5,1, DIRECT);

// motor B
#ifdef USE_MOTOR_B
  #define B_EN_PWM 10
  #define B_DIR_1 11
  #define B_DIR_2 12
  
  #define B_ENCODER_A 2
  #define B_ENCODER_B 4

  double SetpointB, InputB, OutputB = 0.0;
  short int lastPwmB = 0;
  short int maxPwmB = 1024;
  
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

#ifdef DEBUG
  Serial.begin(9600);
#endif
  DEBUG_PRINT("go go");
  pinMode(13, OUTPUT);
  myPIDA.SetSampleTime(SAMPLETIME);
  myPIDA.SetTunings(8,0,0.6);
  myPIDA.SetOutputLimits(-maxPwmA, maxPwmA);
#ifdef USE_MOTOR_B
  myPIDB.SetSampleTime(SAMPLETIME);
  myPIDB.SetTunings(8,0,0.6);
  myPIDB.SetOutputLimits(-maxPwmB, maxPwmB);
#endif
}

char bridePwm = 0;

void pwmWatchdog(char reset = 0){
  static unsigned long lastTime = 0;
  char newbridePwm;
  if (reset) {
    lastTime = micros();
  }
  unsigned long time = micros();
  if ((time - lastTime) > 1000000){
    newbridePwm = 1;
    if ((time - lastTime) > 90000000)
      newbridePwm = 2;
    if (bridePwm != newbridePwm){
      bridePwm = newbridePwm;
      setPwmA(lastPwmA);
      setPwmB(lastPwmB);
    }
  } else {
    bridePwm = 0;
  }
}

int pos=0;
unsigned int cnt;
void loop() {
  InputA = myEncA.read();
  if (myPIDA.Compute())
    setPwmA(OutputA);
  
  InputB = myEncB.read();
  if (myPIDB.Compute())
    setPwmB(OutputB);
    
   handleTask();
   pwmWatchdog();
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
  if (pwm >= 0){
    digitalWrite(dir1_pin, LOW);
    digitalWrite(dir2_pin, HIGH);   
  } else {
    digitalWrite(dir1_pin, HIGH);
    digitalWrite(dir2_pin, LOW);
    pwm = -pwm;
  }
  if (pwm < DEADBAND) pwm = 0;
  if (bridePwm){
    if (pwm > BRIDE_PWM)
      pwm = BRIDE_PWM;
      if (bridePwm == 2)
        pwm = 0;
  }
  Timer1.setPwmDuty(pwm_pin,pwm);
}


inline void setPwmA(short int pwm) {
  DEBUG_PRINT("pwmA ");
  DEBUG_PRINTLN(pwm);
  noInterrupts();
  lastPwmA = pwm;
  interrupts();
  setPwm(pwm, A_EN_PWM, A_DIR_1, A_DIR_2);
}

inline void setPwmB(short int pwm) {
  DEBUG_PRINT("pwmB ");
  DEBUG_PRINTLN(pwm);
  noInterrupts();
  lastPwmB = pwm;
  interrupts();
  setPwm(pwm, B_EN_PWM, B_DIR_1, B_DIR_2);
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
#define CMD_ENABLE_B  0xA
#define CMD_MAX_PWM_AB 0xB
#define CMD_MAX_PWM_B  0xC
#define CMD_INVERT_AB 0xD
#define CMD_INVERT_B  0xE




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
  if (reg > CMD_INVERT_B) return; /* read register */
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

char handleTask() {
  float kp, ki, kd;
  long value, input;
  if (c_buff.remain() <= 0) return 0;
  pwmWatchdog(1);
  Task t = c_buff.pop();
  uint8_t* buff_ptr = t.buffer;
  DEBUG_PRINT("handle task ");
  DEBUG_PRINTLN((int)*buff_ptr);
  int cmd = *buff_ptr++;
  switch(cmd){
  case CMD_PWM_AB:
    if (t.len < 3) break;
    DEBUG_PRINT("pwmA ");
    DEBUG_PRINTLN(readShort(buff_ptr));
    setPwmA(readShort(buff_ptr));
    buff_ptr += 2;
    t.len -= 2;
    // no beak;

#ifdef USE_MOTOR_B
  case CMD_PWM_B:
    if (t.len < 3) break;
    DEBUG_PRINT("pwmB ");
    DEBUG_PRINTLN(readShort(buff_ptr));
    setPwmB(readShort(buff_ptr));
#endif
    break;

  case CMD_RST_AB:
    if (t.len < 5) break;
    DEBUG_PRINT("posA ");
    DEBUG_PRINTLN(readLong(buff_ptr));
    value = readLong(buff_ptr);
    myEncA.write(value);
    SetpointA = value;
    buff_ptr += 4;
    t.len -= 4;
    // no beak;

#ifdef USE_MOTOR_B
  case CMD_RST_B:
    if (t.len < 5) break;
    DEBUG_PRINT("posB ");
    DEBUG_PRINTLN(readLong(buff_ptr));
    value = readLong(buff_ptr);
    myEncB.write(value);
    SetpointB = value; 
    
#endif
    break;

  case CMD_PID_AB:
    if (t.len < 13) break;
    DEBUG_PRINT("pidA ");
    DEBUG_PRINT(readFloat(buff_ptr));
    kp = readFloat(buff_ptr);
    buff_ptr += 4;
    DEBUG_PRINT(" ");
    DEBUG_PRINT(readFloat(buff_ptr));
    ki = readFloat(buff_ptr);
    buff_ptr += 4;
    DEBUG_PRINT(" ");
    DEBUG_PRINTLN(readFloat(buff_ptr));
    kd = readFloat(buff_ptr);
    myPIDA.SetTunings(kp ,ki, kd);
    break;

#ifdef USE_MOTOR_B
  case CMD_PID_B:
    if (t.len < 13) break;
    DEBUG_PRINT("pidB ");
    DEBUG_PRINT(readFloat(buff_ptr));
    kp = readFloat(buff_ptr);
    buff_ptr += 4;
    DEBUG_PRINT(" ");
    DEBUG_PRINT(readFloat(buff_ptr));
    ki = readFloat(buff_ptr);
    buff_ptr += 4;
    DEBUG_PRINT(" ");
    DEBUG_PRINTLN(readFloat(buff_ptr));
    kd = readFloat(buff_ptr);
    myPIDB.SetTunings(kp ,ki, kd);
#endif
    break;

  case CMD_GOTO_AB:
    if (t.len < 5) break;
    DEBUG_PRINT("targetA ");
    DEBUG_PRINTLN(readLong(buff_ptr));
    SetpointA = readLong(buff_ptr);
    myPIDA.SetMode(AUTOMATIC);
    buff_ptr += 4;
    t.len -= 4;
    // no beak;

#ifdef USE_MOTOR_B
  case CMD_GOTO_B:
    if (t.len < 5) break;
    DEBUG_PRINT("targetB ");
    DEBUG_PRINTLN(readLong(buff_ptr));
    SetpointB = readLong(buff_ptr);
    myPIDB.SetMode(AUTOMATIC);
#endif
    break;

  case CMD_ENABLE_AB:
    if (t.len < 2) break;
    if (*buff_ptr){
      DEBUG_PRINTLN("enA ");
      myPIDA.SetMode(AUTOMATIC);
    }
    else {
      DEBUG_PRINTLN("disA ");
      myPIDA.SetMode(MANUAL);
      setPwmA(0);
    }
    buff_ptr += 1;
    t.len -= 1;
    // no beak;

#ifdef USE_MOTOR_B
  case CMD_ENABLE_B:
    if (t.len < 2) break;
    if (*buff_ptr){
      DEBUG_PRINTLN("enB ");
      myPIDB.SetMode(AUTOMATIC);
    }
    else {
      DEBUG_PRINTLN("disB ");
      myPIDB.SetMode(MANUAL);
      setPwmB(0);
    }
#endif
    break;
    
  case CMD_MAX_PWM_AB:
    if (t.len < 3) break;
    DEBUG_PRINT("maxA ");
    DEBUG_PRINTLN(readShort(buff_ptr));
    maxPwmA = readShort(buff_ptr);
    if (maxPwmA <= 0)
      maxPwmA = 0;
    myPIDA.SetOutputLimits(-maxPwmA, maxPwmA);
    buff_ptr += 2;
    t.len -= 2;
    // no beak;

#ifdef USE_MOTOR_B
  case CMD_MAX_PWM_B:
    if (t.len < 3) break;
    DEBUG_PRINT("maxB ");
    DEBUG_PRINTLN(readShort(buff_ptr));
    maxPwmB = readShort(buff_ptr);
    if (maxPwmB <= 0)
      maxPwmB = 0;
    myPIDB.SetOutputLimits(-maxPwmB, maxPwmB);
#endif
    break;

  case CMD_INVERT_AB:
    if (t.len < 2) break;
    if (*buff_ptr){
      DEBUG_PRINTLN("directA ");
      myPIDA.SetControllerDirection(REVERSE);
    }
    else {
      DEBUG_PRINTLN("revA ");
      myPIDA.SetControllerDirection(DIRECT);
    }
    buff_ptr += 1;
    t.len -= 1;
    // no beak;

#ifdef USE_MOTOR_B
  case CMD_INVERT_B:
    if (t.len < 2) break;
    if (*buff_ptr){
      DEBUG_PRINTLN("directB ");
      myPIDB.SetControllerDirection(REVERSE);
    }
    else {
      DEBUG_PRINTLN("revB ");
      myPIDB.SetControllerDirection(DIRECT);
    }
#endif
    break;
    
  default:
    break;
  }
  return 1;
}

/* read request */
#define CMD_GET_POS_A     0x21
#define CMD_GET_POS_B     0x22
#define CMD_GET_PWM_A 0x23
#define CMD_GET_PWM_B 0x24

void requestEvent() {
  long value;
  toogle();
  switch (reg){
  case CMD_GET_POS_A:
    value = myEncA.encoder.position;
    Wire.write((uint8_t *)(&value), 4);
    break;

  case CMD_GET_POS_B:
#ifdef USE_MOTOR_B
    value = myEncB.encoder.position;
    Wire.write((uint8_t *)(&value), 4);
#endif
  break;

  case CMD_GET_PWM_A:
    Wire.write((uint8_t *)(&lastPwmA), 2);
    break;

  case CMD_GET_PWM_B:
#ifdef USE_MOTOR_B
    Wire.write((uint8_t *)(&lastPwmB), 2);
#endif
  break;

  default:
  break;
  }
}
