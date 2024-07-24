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
void updateEncoder();
void ISRUpdateEncoder();
void ISRButton();


//NEED TO FIX THIS VARIABLE
bool buttonPressed = false;
//NEED TO FIX THIS VARIABLE


/*  PID Control Values  */

int setVal = 32;

int measuredVal;

double plateError = 0.0;
double lastPlateError = 0.0;

double MAX_I = 140;

double p_plate_Val, d_plate_Val, i_plate_Val;

double g_plate_Val ;


/*  Object declerations  */

movement::Motor MotorL(MotorL_P1, MotorL_P2);//, &encoder1);
movement::Motor MotorR(MotorR_P1, MotorR_P2);

void setup() {

  SerialPort.begin(115200);



	// Setup Serial Monitor
	Serial.begin(115200);
  Serial.println("Hello" + String(BOARD_NAME));


  /*  Motor Pins  */
  pinMode(MotorL.getPinA(), OUTPUT);
  pinMode(MotorL.getPinB(), OUTPUT);

  pinMode(MotorR.getPinA(), OUTPUT);
  pinMode(MotorR.getPinB(), OUTPUT);


}


void loop() {


  SerialPort.print("motor.Obj Counter: ");
	SerialPort.println(MotorL.encoder->getIncrements() );

  Serial.print("motor.Obj Counter: ");
	Serial.println(MotorL.encoder->getIncrements() );



  int setVal = 32;
  int measuredVal;


  int readVal = 300;

  setVal = map(readVal, 0, 1023, -500, 500);

  measuredVal = MotorL.encoder->getIncrements();

  plateError = setVal - measuredVal;
  

  double PLATE_PID_TOTAL_GAIN = 1.0;
  double P_PLATE_GAIN = 0.55;//1.4 goes very slowl
  double I_PLATE_GAIN = 0.0;
  double D_PLATE_GAIN = 0;

  p_plate_Val = P_PLATE_GAIN *plateError;
  d_plate_Val = D_PLATE_GAIN * (plateError - lastPlateError);
  i_plate_Val = I_PLATE_GAIN *plateError + i_plate_Val; //const *plateError + previous int value
  if (i_plate_Val > MAX_I) {i_plate_Val = MAX_I;}
  if (i_plate_Val < -MAX_I) {i_plate_Val = -MAX_I;}

  g_plate_Val = PLATE_PID_TOTAL_GAIN * ( p_plate_Val + i_plate_Val + d_plate_Val ); 
  lastPlateError =plateError; 

  //Do motor code here
 
 

  /*  SerialPort & Serial Monitor prints  */
  {

  SerialPort.println("g: " + String( g_plate_Val ) );
  Serial.println("g: " + String( g_plate_Val ) );

  SerialPort.println( "error: " + String(plateError ));
  SerialPort.println( "p: " + String( p_plate_Val ));
  Serial.println( "error: " + String(plateError ));
  Serial.println( "p: " + String( p_plate_Val ));

  }

}