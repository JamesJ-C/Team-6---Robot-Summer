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

double p_arm_Val, d_arm_Val, i_arm_Val;

double g_arm_Val ;


/*  Object declerations  */

encoder::RotaryEncoder armEncoder(PB_8, PB_9);
movement::Motor armMotor(ARM_Motor_P1, ARM_MOTOR_P2, &armEncoder);

void setup() {

  SerialPort.begin(115200);



	// Setup Serial Monitor
	Serial.begin(115200);
  Serial.println("Hello" + String(BOARD_NAME));

  /*  Pot Pin  */
  pinMode(POT_PIN, INPUT);


  /*  Motor Pins  */
  pinMode(armMotor.getPinA(), OUTPUT);
  pinMode(armMotor.getPinB(), OUTPUT);



  armMotor.off();
  delay(500);
  armMotor.forward(3000);
  delay(100);
  armMotor.off();
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

  measuredVal = armMotor.encoder->getIncrements();

  plateError = setVal - measuredVal;
  

  double PLATE_PID_TOTAL_GAIN = 1.0;
  double P_ARM_GAIN = 0.55;//1.4 goes very slowl
  double I_ARM_GAIN = 0.0;
  double D_ARM_GAIN = 0;

  p_arm_Val = P_ARM_GAIN *plateError;
  d_arm_Val = D_ARM_GAIN * (plateError - lastPlateError);
  i_arm_Val = I_ARM_GAIN *plateError + i_arm_Val; //const *plateError + previous int value
  if (i_arm_Val > MAX_I) {i_arm_Val = MAX_I;}
  if (i_arm_Val < -MAX_I) {i_arm_Val = -MAX_I;}

  g_arm_Val = PLATE_PID_TOTAL_GAIN * ( p_arm_Val + i_arm_Val + d_arm_Val ); 
  lastPlateError =plateError; 

// PID hopefully
  armMotor.forward( g_arm_Val );

}

/**
 * @brief function attached to RotaryA and RotaryB to update encoder values
 * 
 */
void ISRUpdateEncoder(){

  bool A = digitalRead(ROTARY_A);
  bool B = digitalRead(ROTARY_B);

  armEncoder.updateEncoder(A, B);
  armEncoder.updateTime( millis() );

}


/**
 * @brief function for reading the debounced button press values.
 * 
 */
void ISRButton() {

  //  Serial.print("inside the interrupt");

  armEncoder.resetIncrement();
  //delay(100);
  buttonPressed = true;

  //armMotor.buttonPressed = true;

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
    while (!digitalRead(LOWER_LIMIT_ARM)) {
        armMotor.backward(2000);
    }

    // initialize bottom of elevator movement
    armMotor.off();
    armEncoder.resetIncrement();
    bottom = armEncoder.getIncrements();

    // turn motor in opposite direction until top limit reached
    while (!digitalRead(UPPER_LIMIT_ARM)) {
        armMotor.forward(2000);
    }

    // initialize top of elevator movement
    armMotor.off();
    top = encoder1.getIncrements();
    encoder1.setMaxIncrement(top);

    // turn motor and reach middle of motion
    center = top / 2;
    while (encoder1.getIncrements() != center) {
        armMotor.backward(3000);
    }
    armMotor.stop();

}