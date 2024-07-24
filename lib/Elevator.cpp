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
void localize();


//NEED TO FIX THIS VARIABLE
bool buttonPressed = false;
//NEED TO FIX THIS VARIABLE


/*  PID Control Values  */

int setVal = 32;

int measuredVal;

double plateError = 0.0;
double lastPlateError = 0.0;

double MAX_I = 140;

double p_elevator_Val, d_elevator_Val, i_elevator_Val;

double g_elevator_Val ;


/*  Object declerations  */

encoder::RotaryEncoder encoder1(PB_8, PB_9);
movement::Motor MotorElevator(MotorElevator_P1, MotorElevator_P2);//, &encoder1);

void setup() {

  SerialPort.begin(115200);



	// Setup Serial Monitor
	Serial.begin(115200);
  Serial.println("Hello" + String(BOARD_NAME));

  /*  Pot Pin  */
  pinMode(POT_PIN, INPUT);


  /*  Motor Pins  */
  pinMode(MotorElevator.getPinA(), OUTPUT);
  pinMode(MotorElevator.getPinB(), OUTPUT);



  MotorElevator.off();
  delay(500);
  MotorElevator.forward(3000);
  delay(100);
  MotorElevator.off();
  delay(100);

  /*  Encoders  */
	pinMode(ROTARY_A, INPUT);
	pinMode(ROTARY_B, INPUT);
  // pinMode(encoder1.getPinA(), INPUT);
	// pinMode(encoder1.getPinB(), INPUT);

  pinMode(BUTTON_PIN, INPUT);


  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), ISRButton, RISING);

    attachInterrupt(digitalPinToInterrupt(ROTARY_A), ISRUpdateEncoder, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ROTARY_B), ISRUpdateEncoder, CHANGE);

    // perform motor sweep to initialize motion
    localize();

}


void loop() {

  int setVal = 32;
  int measuredVal;


  int readVal = analogRead(POT_PIN);

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
  MotorElevator.forward( g_elevator_Val );

}

/**
 * @brief function attached to RotaryA and RotaryB to update encoder values
 * 
 */
void ISRUpdateEncoder(){

  bool A = digitalRead(ROTARY_A);
  bool B = digitalRead(ROTARY_B);

  encoder1.updateEncoder(A, B);
  encoder1.updateTime( millis() );

}


/**
 * @brief function for reading the debounced button press values.
 * 
 */
void ISRButton() {

  //  Serial.print("inside the interrupt");

  encoder1.resetIncrement();
  //delay(100);
  buttonPressed = true;

  //MotorElevator.buttonPressed = true;

}

/**
 * @brief performs motor sweep to localize range of elevator motion 
 * updates encoder range values and sends the motor to the center once completed
 * 
 */
void localize() {

    int bottom;
    int top;
    int center;

    // turn motor until elevator reaches bottom limit
    while (!digitalRead(LOWER_LIMIT_ELEVATOR)) {
        MotorElevator.backward(2000);
    }

    // initialize bottom of elevator movement
    MotorElevator.off();
    MotorElevator.encoder->resetIncrement();
    bottom = MotorElevator.encoder->getIncrements();

    // turn motor in opposite direction until top limit reached
    while (!digitalRead(UPPER_LIMIT_ELEVATOR)) {
        MotorElevator.forward(2000);
    }

    // initialize top of elevator movement
    MotorElevator.off();
    top = MotorElevator.encoder->getIncrements();
    MotorElevator.encoder->setMaxIncrement(top);

    // turn motor and reach middle of motion
    center = top / 2;
    while (MotorElevator.encoder->getIncrements() != center) {
        MotorElevator.backward(3000);
    }
    MotorElevator.stop();

}