#include <Arduino.h>
#include <Wire.h>
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


/*  Function Declerations  */
void ISRUpdateElevatorEncoder();
void localize();

/*  PID Control Values  */

int setVal = 32;

int measuredVal;

double plateError = 0.0;
double lastPlateError = 0.0;

double MAX_I = 140;

double p_elevator_Val, d_elevator_Val, i_elevator_Val;

double g_elevator_Val ;

const int motorSpeed = 2000;


/*  Object declerations  */

encoder::RotaryEncoder elevatorEncoder( ELEVATOR_ROTARY_ENCODER_A, ELEVATOR_ROTARY_ENCODER_B);
movement::Motor MotorElevator(MOTOR_ELEVATOR_P1, MOTOR_ELEVATOR_P2, &elevatorEncoder);

void setup() {
  delay(2000);

  SerialPort.begin(115200);



	// Setup Serial Monitor
	Serial.begin(115200);
  Serial.println("Hello" + String(BOARD_NAME));


  /*  Motor Pins  */
  pinMode(MotorElevator.getPinA(), OUTPUT);
  pinMode(MotorElevator.getPinB(), OUTPUT);


  /*  Encoders  */
	pinMode(elevatorEncoder.getPinA(), INPUT);
	pinMode(elevatorEncoder.getPinB(), INPUT);


  attachInterrupt(digitalPinToInterrupt( elevatorEncoder.getPinA() ), ISRUpdateElevatorEncoder, CHANGE);
	attachInterrupt(digitalPinToInterrupt( elevatorEncoder.getPinB() ), ISRUpdateElevatorEncoder, CHANGE);



  // MotorElevator.off();
  // delay(500);
  // MotorElevator.forward(3000);
  // delay(100);
  // MotorElevator.off();
  // delay(100);
  // perform motor sweep to initialize motion
  localize();

}


void loop() {

  int setVal = 32;
  int measuredVal;


  int readVal = SerialPort.read();

  setVal = map(readVal, 0, 1023, -500, 500);

  measuredVal = MotorElevator.encoder->getIncrements();

  plateError = setVal - measuredVal;
  

  double PLATE_PID_TOTAL_GAIN = 1.0;
  double P_ELEVATOR_GAIN = 0.55;//1.4 goes very slowl
  double I_ELEVATOR_GAIN = 0.0;
  double D_ELEVATOR_GAIN = 0;

  p_elevator_Val = P_ELEVATOR_GAIN *plateError;
  d_elevator_Val = D_ELEVATOR_GAIN * (plateError - lastPlateError);
  i_elevator_Val = I_ELEVATOR_GAIN *plateError + i_elevator_Val; //const *plateError + previous int value
  if (i_elevator_Val > MAX_I) {i_elevator_Val = MAX_I;}
  if (i_elevator_Val < -MAX_I) {i_elevator_Val = -MAX_I;}

  g_elevator_Val = PLATE_PID_TOTAL_GAIN * ( p_elevator_Val + i_elevator_Val + d_elevator_Val ); 
  lastPlateError =plateError; 

// PID hopefully
  // MotorElevator.forward( g_elevator_Val );

}

/**
 * @brief function attached to RotaryA and RotaryB to update encoder values
 * 
 */
void ISRUpdateElevatorEncoder(){

  bool A = digitalRead( elevatorEncoder.getPinA() );
  bool B = digitalRead( elevatorEncoder.getPinB() );

  elevatorEncoder.updateEncoder(A, B);
  elevatorEncoder.updateTime( millis() );

}

/**
 * @brief performs motor sweep to localize range of elevator motion 
 * updates encoder range values and sends the motor to the center once completed
 * 
 */
void localize() {
    Serial.println('beginning localize function'); 
    int bottom;
    int top;
    int center;

    // turn motor until elevator reaches bottom limit
    while (!digitalRead(ELEVATOR_LOWER_LIMIT_SWITCH)) {
        MotorElevator.backward(motorSpeed);
    }

    // initialize bottom of elevator movement
    MotorElevator.off();
    MotorElevator.encoder->resetIncrement();
    bottom = MotorElevator.encoder->getIncrements();

    // turn motor in opposite direction until top limit reached
    while (!digitalRead(ELEVATOR_UPPER_LIMIT_SWITCH)) {
        MotorElevator.forward(motorSpeed);
    }

    // initialize top of elevator movement
    MotorElevator.off();
    top = MotorElevator.encoder->getIncrements();
    MotorElevator.encoder->setMaxIncrement(top);

    // turn motor and reach middle of motion
    center = top / 2;
    while (MotorElevator.encoder->getIncrements() != center) {
        MotorElevator.backward(motorSpeed);
    }
    MotorElevator.stop();

}