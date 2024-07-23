#include <Arduino.h>
#include <Servo.h>
#include <HardwareSerial.h>
#include "RotaryEncoder.h"
#include "Motor.h"
#include "robotConstants.h"

/*  BP pin defs for UART */

#define RX PB11
#define TX PB10

HardwareSerial SerialPort(USART3);
String msg;

#define PLATE_MOTOR_PIN_A PB_8
#define PLATE_MOTOR_PIN_B PB_9
#define PLATE_ENCODER_PIN_A PB_1
#define PLATE_ENCODER_PIN_B PB_2
#define LIMIT_SWITCH_LEFT PB12
#define LIMIT_SWITCH_RIGHT PB15
#define SERVOPIN PB13

movement::Motor motor(PLATE_MOTOR_PIN_A, PLATE_MOTOR_PIN_B);
encoder::RotaryEncoder plateencoder(PLATE_ENCODER_PIN_A, PLATE_ENCODER_PIN_B);

Servo plateservo; 

volatile int encoderPosition = 0; // volatile tells compllier that the value can be changed by smth outside of normal program flow like ISR or external hardware
bool movingRight = true; 

//function prototypes
void checkLimitSwitches(); 
void ISRUpdateEncoder();

void setup(){

  Serial.begin(115200); 
  //Motor Pins
  pinMode(PLATE_MOTOR_PIN_A, OUTPUT); 
  pinMode(PLATE_MOTOR_PIN_B, OUTPUT);

 //Encoder Pins 
  pinMode(PLATE_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(PLATE_ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PLATE_ENCODER_PIN_A), ISRUpdateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PLATE_ENCODER_PIN_B), ISRUpdateEncoder, CHANGE);
  //Sets up interrupt to call 'updateEncoder' ISR function whenever theres a change to the encoder pin 
  
  //Limit Switch Pins 
  pinMode(LIMIT_SWITCH_LEFT, INPUT);
  pinMode(LIMIT_SWITCH_RIGHT, INPUT);

  plateservo.attach(SERVOPIN);
  plateservo.write(90);

  motor.forward(4096); 
}
void loop(){
  //Serial.print("Encoder Position: ");
  //Serial.println(encoderPosition);
  //delay(100);
  checkLimitSwitches();
}

  void checkLimitSwitches() {
  if (digitalRead(LIMIT_SWITCH_LEFT) == LOW && !movingRight) {
    SerialPort.println("off1");
    motor.off();
  } else if (digitalRead(LIMIT_SWITCH_RIGHT) == LOW && movingRight) {
    motor.off();
    SerialPort.println("off1");
    delay(10000);
    motor.backward(4096); // Start moving backward
    movingRight = false;
  }
}

void ISRUpdateEncoder(){

  bool A = digitalRead(ROTARY_A);
  bool B = digitalRead(ROTARY_B);

  plateencoder.updateEncoder(A, B);
  plateencoder.updateTime( millis());
}