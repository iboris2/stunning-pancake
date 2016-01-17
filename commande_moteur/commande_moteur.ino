#include <Wire.h>
#include <Encoder.h>
#include <TimerOne.h>

#define I2C_ADDRESS 8

#define USE_MOTOR_B
// motor A
#define A_EN_PWM 9
#define A_DIR_1 7
#define A_DIR_2 8

#define A_ENCODER_A 2
#define A_ENCODER_B 4

// motor B
#ifdef USE_MOTOR_B
  #define B_EN_PWM 10
  #define B_DIR_1 11
  #define B_DIR_2 12
  
  #define B_ENCODER_A 3
  #define B_ENCODER_B 5
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

void setup() {
  Wire.begin(I2C_ADDRESS);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(requestEvent); // register event

  Timer1.initialize(40);  // 40 us = 25 kHz
  initMotor(A_EN_PWM, A_DIR_1, A_DIR_2);
#ifdef USE_MOTOR_B
  initMotor(B_EN_PWM, B_DIR_1, B_DIR_2);
#endif

  Serial.begin(9600);
  Serial.println("go go");
  pinMode(13, OUTPUT);
}

void loop() {
  delay(50);
  Serial.println(myEncA.read());
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

#define CMD_PWM_AB 0x1
#define CMD_PWM_B  0x2
#define CMD_RST_AB 0x3
#define CMD_RST_B  0x4
char reg = 0x0;

void receiveEvent(int howMany) {
  short int pwm;
  long value;
  // force encoder update to prevent loosing step
  myEncA.read();
#ifdef USE_MOTOR_B
  myEncB.read();
#endif
  if (howMany) reg = Wire.read();
  switch(reg){
  case CMD_PWM_AB:
    if (howMany < 3) break;
    pwm = (short int)Wire.read() | (short int)Wire.read() << 8;
    setPwm(pwm, A_EN_PWM, A_DIR_1, A_DIR_2);
    howMany -= 2;
    // no beak;
    
  case CMD_PWM_B:
#ifdef USE_MOTOR_B
    if (howMany < 3) break;
    pwm = (short int)Wire.read() | (short int)Wire.read() << 8;
    setPwm(pwm, B_EN_PWM, B_DIR_1, B_DIR_2);
#endif
    break;

  case CMD_RST_AB:
    if (howMany < 5) break;
    value = (long)Wire.read() | (long)Wire.read() << 8 | (long)Wire.read() << 16 | (long)Wire.read() << 24;
    myEncA.write(value);
    howMany -= 4;
    // no beak;

  case CMD_RST_B:
#ifdef USE_MOTOR_B
    value = (long)Wire.read() | (long)Wire.read() << 8 | (long)Wire.read() << 16 | (long)Wire.read() << 24;
    myEncB.write(value);
#endif
    break;
    
  default:
    break;
  }
}

void requestEvent() {
  long valueA = myEncA.encoder.position;
  Serial.print("posA ");Serial.println(valueA);
  Wire.write((uint8_t *)(&valueA), 4);
#ifdef USE_MOTOR_B
  long valueB = myEncB.encoder.position;
  Serial.print("posB ");Serial.println(valueB);
  Wire.write((uint8_t *)(&valueB), 4);
#endif
}
