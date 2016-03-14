// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

enum motor_t {
  MOTOR_A,
  MOTOR_B
 };

#define CMD_PWM_AB 0x1
#define CMD_PWM_B  0x2
#define CMD_RST_AB 0x3
#define CMD_RST_B  0x4

class Motor {
  uint8_t slave;
  motor_t motor;
  /* commands */
  uint8_t pwm_cmd;
  uint8_t rst_cmd;
public:
  Motor(uint8_t p_slave, enum motor_t p_motor) {
    slave = p_slave;
    motor = p_motor;
    if (motor == MOTOR_A) { 
      pwm_cmd = CMD_PWM_AB;
      rst_cmd = CMD_RST_AB;
    } else {
      pwm_cmd = CMD_PWM_B;
      rst_cmd = CMD_RST_B;   
    }
  }

  void pwm(short int value){
    Wire.beginTransmission(slave); // transmit to device #8
    Wire.write(pwm_cmd);        // sends five bytes
    Wire.write((uint8_t* ) &value,2);
    Wire.endTransmission();
  }

  void reset(long value){
    Wire.beginTransmission(slave); // transmit to device #8
    Wire.write(rst_cmd);        // sends five bytes
    Wire.write((uint8_t* ) &value,4);
    Wire.endTransmission();
  }

  long position(){
    static long storedValueA = 0;
    static long storedValueB = 0;
    int req_len;
    Wire.requestFrom(8, 8);    // request 4 bytes from slave device #8
    if (Wire.available() != 8) {
      while(Wire.available()) {
        Wire.read(); // flush
      }
    } else {
      storedValueA = (long)Wire.read() | (long)Wire.read() << 8 | (long)Wire.read() << 16 | (long)Wire.read() << 24;
      storedValueB = (long)Wire.read() | (long)Wire.read() << 8 | (long)Wire.read() << 16 | (long)Wire.read() << 24; 
    }
    if (motor == MOTOR_A) { 
      return storedValueA;
    } else {
      return storedValueB;   
    }
  }
};

void setup() {
  pinMode(13, OUTPUT);
  //pinMode(A4, INPUT_PULLUP);
 // pinMode(A5, INPUT_PULLUP);
  Wire.begin();        // join i2c bus (address optional for master)
  Wire.setClock(50000L);
  Serial.begin(9600);
}



short int i = 0;
char dir = 0;
long value = 0;

Motor Motor_Z(8, MOTOR_B);
Motor Motor_X(8, MOTOR_A);
Motor Motor_Y(10, MOTOR_A);


void loop() {

  if (Serial.available()) {
    switch((char)Serial.read()){
      case 'u': Motor_Z.pwm(512);delay(15);Motor_Z.pwm(1023); break;
      case 'd': Motor_Z.pwm(-512);delay(15);Motor_Z.pwm(-1023);break;
      case 's': Motor_Z.pwm(0);break;
    }
    digitalWrite(13, value);
    value = ~value;
    Serial.write('1');
  }
  delay(100);Motor_X.pwm(0);Motor_Y.pwm(0);
  /*Serial.print("Z: ");
  Serial.println(Motor_Z.position());
  Serial.print("X: ");
  Serial.println(Motor_X.position());
  Serial.print("Y: ");
  Serial.println(Motor_Y.position());*/
}




