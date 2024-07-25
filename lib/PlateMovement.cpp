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

enum State {
  IDLE,
  MOVING_RIGHT,
  MOVING_LEFT,
  ACCEPT_BURGER,
  SERVE_BURGER,
  FINISH_SERVE,
  FINISH_ACCEPT,
  TRAVEL_MODE
};
State currentState = IDLE; 


//function prototypes
// void checkLimitSwitches(); // switch goes high when pressed
// void ISRUpdateEncoder();
// void ISRButton();
void openClaw();
void closeClaw();
void movePlateRight();
void movePlateLeft();
void stopPlate();
void sweepObject();
void clawNeutral(); 
void movePlateMiddlefromRight(); 
void movePlateMiddlefromLeft(); 
void checkState(); 

void setup(){
  Serial.begin(115200); 
  Serial.println(); 
  //Motor Pins
  pinMode(PLATE_MOTOR_PIN_A, OUTPUT); 
  pinMode(PLATE_MOTOR_PIN_B, OUTPUT);

 //Encoder Pins 
  pinMode(PLATE_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(PLATE_ENCODER_PIN_B, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(PLATE_ENCODER_PIN_A), ISRUpdateEncoder, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PLATE_ENCODER_PIN_B), ISRUpdateEncoder, CHANGE);
  //Sets up interrupt to call 'updateEncoder' ISR function whenever theres a change to the encoder pin 
  
  //Limit Switch Pins 
  pinMode(LIMIT_SWITCH_LEFT, INPUT);
  pinMode(LIMIT_SWITCH_RIGHT, INPUT);
  // attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_LEFT),ISRButton, RISING);
  // attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_RIGHT),ISRButton, RISING);

  //Attaching servos 
  plateservo.attach(PLATESERVO);
  clawservoleft.attach(CLAWSERVO1); 
  clawservoright.attach(CLAWSERVO2);

//Initial Claw Position
  clawNeutral(); 

}
void loop(){
  checkState(); 
switch (currentState)
{
case IDLE:
  break;

case MOVING_RIGHT:
  movePlateRight();
  break; 

case SERVE_BURGER:
  stopPlate();
  sweepObject(); 
  currentState = FINISH_SERVE; 
  break; 

case FINISH_SERVE:
  clawNeutral(); 
  movePlateMiddlefromRight(); 
  currentState = IDLE; //IDLE STATE is with plate in the middle and claw in neutral position 

case MOVING_LEFT:
  movePlateLeft(); 

case ACCEPT_BURGER:
  stopPlate();
  openClaw();

case TRAVEL_MODE:
  closeClaw(); 
  delay(2000);
  void movePlateMiddlefromLeft();
  currentState = IDLE; 
  }
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

void checkState(){
  if(inservingarea){
  currentState = MOVING_RIGHT;
} else if(digitalRead(LIMIT_SWITCH_RIGHT == HIGH)){
  currentState = SERVE_BURGER; 
} else if(atcuttingarea){
  currentState = MOVING_LEFT; 
} else if(digitalRead(LIMIT_SWITCH_RIGHT == HIGH)){
  currentState = ACCEPT_BURGER; 
} else if(wallesaysdone){
  currentState = TRAVEL_MODE; 
}
}

// void ISRUpdateEncoder(){

//   bool A = digitalRead(ROTARY_A);
//   bool B = digitalRead(ROTARY_B);

//   plateencoder.updateEncoder(A, B);
//   plateencoder.updateTime( millis());
// }

// void ISRButton(){
//   motor.off(); 
//   movingRight = !movingRight; 
//   // Serial.println(String(count) + "ISR function");
//   //delay(2000);
// }

void stopPlate(){
  motor.stop(); 
}

void movePlateRight(){
  motor.backward(3500); 
}

void movePlateLeft(){
  motor.forward(3500); 
}

void expandPlate(){
  plateservo.write(90);
  delay(1000); 
}

void movePlateMiddlefromRight(){
  motor.forward(3500);
  delay(2500); 
  motor.off(); 
}

void movePlateMiddlefromLeft(){
  motor.backward(3500); 
  delay(2500);
  motor.off();
}

void openClaws(){
  clawservoleft.write(0); 
  clawservoright.write(180);
}

void closeClaws(){
  clawservoleft.write(90); 
  clawservoright.write(90);
}

void clawNeutral(){
  clawservoleft.write(30);
  clawservoright.write(40); 
}

void sweepObject(){
  clawservoright.write(180); 
  delay(1000);
  clawservoleft.write(180); 
}


