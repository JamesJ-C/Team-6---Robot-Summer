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

double p_lazySusan_Val, d_lazySusan_Val, i_lazySusan_Val;

double g_lazySusan_Val ;


/*  Object declerations  */

encoder::RotaryEncoder elevatorEncoder( LAZY_SUSAN_ROTARY_ENCODER_A, LAZY_SUSAN_ROTARY_ENCODER_B);
movement::Motor lazySusanMotor(LAZY_SUSAN_MOTOR_P1, LAZY_SUSAN_MOTOR_P2, &elevatorEncoder);

void setup() {

  SerialPort.begin(115200);



	// Setup Serial Monitor
	Serial.begin(115200);
  Serial.println("Hello" + String(BOARD_NAME));


  /*  Motor Pins  */
  pinMode(lazySusanMotor.getPinA(), OUTPUT);
  pinMode(lazySusanMotor.getPinB(), OUTPUT);


  /*  Encoders  */
	pinMode(elevatorEncoder.getPinA(), INPUT);
	pinMode(elevatorEncoder.getPinB(), INPUT);


  attachInterrupt(digitalPinToInterrupt( elevatorEncoder.getPinA() ), ISRUpdateElevatorEncoder, CHANGE);
	attachInterrupt(digitalPinToInterrupt( elevatorEncoder.getPinB() ), ISRUpdateElevatorEncoder, CHANGE);



  // lazySusanMotor.off();
  // delay(500);
  // lazySusanMotor.forward(3000);
  // delay(100);
  // lazySusanMotor.off();
  // delay(100);
  // perform motor sweep to initialize motion
  //localize();

}


void loop() {


  lazySusanMotor.forward(2700);
  delay(200);
  lazySusanMotor.off();
  lazySusanMotor.backward(2700);
  delay(200);
  lazySusanMotor.off();
  delay(200);

  //localize();

////

//   delay(1000);
//   SerialPort.println("loop");

//   int setVal = 32;
//   int measuredVal;


//   int readVal = SerialPort.read();

//   setVal = map(readVal, 0, 1023, -500, 500);

//   measuredVal = lazySusanMotor.encoder->getIncrements();

//   plateError = setVal - measuredVal;
  

//   double PLATE_PID_TOTAL_GAIN = 1.0;
//   double P_LAZY_SUSAN_GAIN = 0.55;//1.4 goes very slowl
//   double I_LAZY_SUSAN_GAIN = 0.0;
//   double D_LAZY_SUSAN_GAIN = 0;

//   p_lazySusan_Val = P_LAZY_SUSAN_GAIN *plateError;
//   d_lazySusan_Val = D_LAZY_SUSAN_GAIN * (plateError - lastPlateError);
//   i_lazySusan_Val = I_LAZY_SUSAN_GAIN *plateError + i_lazySusan_Val; //const *plateError + previous int value
//   if (i_lazySusan_Val > MAX_I) {i_lazySusan_Val = MAX_I;}
//   if (i_lazySusan_Val < -MAX_I) {i_lazySusan_Val = -MAX_I;}

//   g_lazySusan_Val = PLATE_PID_TOTAL_GAIN * ( p_lazySusan_Val + i_lazySusan_Val + d_lazySusan_Val ); 
//   lastPlateError =plateError; 

// // PID hopefully
//   // lazySusanMotor.forward( g_lazySus_Val );

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

  const int motorSpeed = 2000;

    int bottom;
    int top;
    int center;

    // turn motor until elevator reaches bottom limit
    while (!digitalRead(LAZY_SUSAN_LOWER_LIMIT_SWITCH)) {
        lazySusanMotor.backward(motorSpeed);
        SerialPort.println("backward");
    }

    // initialize bottom of elevator movement
    lazySusanMotor.off();
    lazySusanMotor.encoder->resetIncrement();
    bottom = lazySusanMotor.encoder->getIncrements();

    // turn motor in opposite direction until top limit reached
    while (!digitalRead(LAZY_SUSAN_UPPER_LIMIT_SWITCH)) {
        lazySusanMotor.forward(motorSpeed);
        SerialPort.println("forward");
    }

    // initialize top of elevator movement
    lazySusanMotor.off();
    top = lazySusanMotor.encoder->getIncrements();
    lazySusanMotor.encoder->setMaxIncrement(top);

    // turn motor and reach middle of motion
    center = top / 2;
    while (lazySusanMotor.encoder->getIncrements() != center) {
        lazySusanMotor.backward(motorSpeed);
        SerialPort.println("center");
    }
    lazySusanMotor.stop();

}