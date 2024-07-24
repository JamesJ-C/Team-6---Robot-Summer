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
#define PLATESERVO PB12
#define CLAWSERVO1 PB13 
#define CLAWSERVO2 PB14
#define CLAW_INITIAL_L 90
#define CLAW_INITIAL_R 90
#define CLAW_OPEN_L 60
#define CLAW_OPEN_R 150

movement::Motor motor(PLATE_MOTOR_PIN_A, PLATE_MOTOR_PIN_B);
encoder::RotaryEncoder plateencoder(PLATE_ENCODER_PIN_A, PLATE_ENCODER_PIN_B);

Servo plateservo; 
Servo clawservoleft; 
Servo clawservoright;

volatile int encoderPosition = 0; // volatile tells compllier that the value can be changed by smth outside of normal program flow like ISR or external hardware
bool movingRight = true; 

//function prototypes
void checkLimitSwitches(); // switch goes high when pressed
void ISRUpdateEncoder();
void ISRButton();
void openClaw();
void closeClaw();

void setup(){
  Serial.begin(115200); 
  Serial.println(); 
  //Motor Pins
  pinMode(PLATE_MOTOR_PIN_A, OUTPUT); 
  pinMode(PLATE_MOTOR_PIN_B, OUTPUT);

 //Encoder Pins 
  pinMode(PLATE_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(PLATE_ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PLATE_ENCODER_PIN_A), ISRUpdateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PLATE_ENCODER_PIN_B), ISRUpdateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_LEFT),ISRButton, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_RIGHT),ISRButton, RISING);

  //Sets up interrupt to call 'updateEncoder' ISR function whenever theres a change to the encoder pin 
  
  //Limit Switch Pins 
  pinMode(LIMIT_SWITCH_LEFT, INPUT);
  pinMode(LIMIT_SWITCH_RIGHT, INPUT);

  plateservo.attach(PLATESERVO);
  plateservo.write(90);

  clawservoleft.attach(CLAWSERVO1); 
  clawservoright.attach(CLAWSERVO2);

//Initial Claw Position
  clawservoleft.write(CLAW_INITIAL_L);
  clawservoright.write(CLAW_INITIAL_R);

}
void loop(){
  //Serial.print("Encoder Position: ");
  //Serial.println(encoderPosition);
  //delay(100);
  // checkLimitSwitches();

if(movingRight == true){
  motor.backward(3500);
}
else{
  motor.forward(3500);
}
  //checking boolean if its one state move in direction if not ,ove in another and interrupt for when stwitch is preessed and th state is sqwitches 

}
  void checkLimitSwitches(){
  if (digitalRead(LIMIT_SWITCH_LEFT) == HIGH && !movingRight) {
    motor.off();
  } else if (digitalRead(LIMIT_SWITCH_RIGHT) == HIGH && movingRight) {
    motor.off();
    delay(2000);
    motor.forward(3500); // Start moving backward
    movingRight = false;
  }
}

void ISRUpdateEncoder(){

  bool A = digitalRead(ROTARY_A);
  bool B = digitalRead(ROTARY_B);

  plateencoder.updateEncoder(A, B);
  plateencoder.updateTime( millis());
}

int count = 0;

void ISRButton(){
  motor.off(); 
  movingRight = !movingRight; 
  // Serial.println(String(count) + "ISR function");
  delay(2000);
}



void 


void openclaw(){
  clawservoleft.write(CLAW_OPEN_L);
  clawservoright.write(CLAW_OPEN_R); 
}

void closeclaw(){
}