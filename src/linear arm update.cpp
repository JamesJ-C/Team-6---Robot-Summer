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
voidlocalizeLazySusan();

void localizeArm();
//void limit();

/*  PID Control Values  */

// int setVal = 32;

int measuredVal;

double lazySusanError = 0.0;
double lastlazySusanError = 0.0;

double MAX_I = 1400;

double p_lazySusan_Val, d_lazySusan_Val, i_lazySusan_Val;

double g_lazySusan_Val ;

bool buttonPushed;

/*  Object declerations  */

encoder::RotaryEncoder elevatorEncoder( LAZY_SUSAN_ROTARY_ENCODER_A, LAZY_SUSAN_ROTARY_ENCODER_B);
movement::Motor lazySusanMotor(LAZY_SUSAN_MOTOR_P1, LAZY_SUSAN_MOTOR_P2, &elevatorEncoder);

void setup() {
  delay(2000);

  SerialPort.begin(115200);



	// Setup Serial Monitor
	Serial.begin(115200);
  Serial.println("Hello" + String(BOARD_NAME));


  /*  Motor Pins  */
  pinMode(lazySusanMotor.getPinA(), OUTPUT);
  pinMode(lazySusanMotor.getPinB(), OUTPUT);

  pinMode(LAZY_SUSAN_LIMIT_SWITCH, INPUT);


  /*  Encoders  */
	pinMode(elevatorEncoder.getPinA(), INPUT_PULLUP);
	pinMode(elevatorEncoder.getPinB(), INPUT_PULLUP);


  attachInterrupt(digitalPinToInterrupt( elevatorEncoder.getPinA() ), ISRUpdateElevatorEncoder, CHANGE);
	attachInterrupt(digitalPinToInterrupt( elevatorEncoder.getPinB() ), ISRUpdateElevatorEncoder, CHANGE);

  //attachInterrupt(digitalPinToInterrupt(LAZY_SUSAN_LIMIT_SWITCH), limit, CHANGE);

  buttonPushed = false;

  // lazySusanMotor.off();
  // delay(500);
  // lazySusanMotor.forward(3000);
  // delay(100);
  // lazySusanMotor.off();
  // delay(100);
  // perform motor sweep to initialize motion
  localizeArm();

}

int loopCount = 0;
int loopSetCount = 0;
int setVal = 50;

void loop() {

Serial.println("Encoder: " + String(
  elevatorEncoder.getIncrements() )

);


  //localizeLazySusan();
  // lazySusanMotor.forward(2000);
  // delay(500);
  // lazySusanMotor.off();
  // delay(500);
  // lazySusanMotor.backward(2000);
  // delay(500);
  // lazySusanMotor.off();
  // delay(500);

  //localizeLazySusan();

////

//   delay(1000);
//   SerialPort.println("loop");

//   int setVal = 32;
//   int measuredVal;


//   int readVal = SerialPort.read();

//   setVal = map(readVal, 0, 1023, -500, 500);

int error = 100 - lazySusanMotor.encoder->getIncrements();
const int motorSpeed = 2700;
int errorCount = 0;
while (lazySusanMotor.encoder->getIncrements() != 100 || errorCount != -1){
  
  if (error < 0){
    lazySusanMotor.setMotor(motorSpeed);
  }
  else if (error > 0){
    lazySusanMotor.setMotor( -1 * motorSpeed);
  }
  else {
    errorCount++;
    Serial.println("break?");
  }
error = 100 - lazySusanMotor.encoder->getIncrements();

Serial.println("encoder: " + String (lazySusanMotor.encoder->getIncrements() ));
Serial.println("error: " + String (error ));


}

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
 * @brief performs motor sweep to localizeLazySusan range of elevator motion 
 * updates encoder range values and sends the motor to the center once completed
 * 
 */
void localizeLazySusan() {

  const int motorSpeed = 2000;

    int start;
    int end;
    int center;

  // initialize start of lazy susan movement
    lazySusanMotor.off();
    lazySusanMotor.encoder->resetIncrement();
    start = lazySusanMotor.encoder->getIncrements();

  String incomimingMsg = "f";
  char a;

    do {
      incomimingMsg = SerialPort.readString();
      a = incomimingMsg.charAt(0);
      lazySusanMotor.forward(motorSpeed);
      Serial.println("do while");
      SerialPort.println("do while");
      SerialPort.println("msg: " + incomimingMsg);
      Serial.println("msg: " + incomimingMsg);
    }
    //while (!digitalRead(LAZY_SUSAN_LIMIT_SWITCH));
    while ( a != 'L' );
    Serial.println("out of loop");
    // initialize end limit of movement
    lazySusanMotor.off();
    delay(1000);
    end = lazySusanMotor.encoder->getIncrements();
    lazySusanMotor.encoder->setMaxIncrement(end);

    // turn motor and reach middle of motion
    center = end / 2;
    while (lazySusanMotor.encoder->getIncrements() != center) {
        lazySusanMotor.backward(motorSpeed);
        SerialPort.println("center");
        Serial.println("center: " + String ( lazySusanMotor.encoder->getIncrements() ) );
    }
    lazySusanMotor.stop();

}

// void limit() {
//   buttonPushed = !buttonPushed;
//   Serial.println("interrupt");
// }


/**
 * @brief performs motor sweep to localizeArm range of elevator motion 
 * updates encoder range values and sends the motor to the center once completed
 * 
 */
void localizeArm() {

    int bottom;
    int top;
    int center;

    // turn motor until elevator reaches bottom limit

    Serial.println("dr1" + String (digitalRead(ARM_LOWER_LIMIT_SWITCH) ) );

    do {
    //while (!digitalRead(ARM_LOWER_LIMIT_SWITCH)) {
        lazySusanMotor.forward(1500);
        Serial.print("1: ");
        Serial.println(digitalRead(ARM_LOWER_LIMIT_SWITCH));
    } while (!digitalRead(ARM_LOWER_LIMIT_SWITCH));
  delay(1000);
    // initialize bottom of elevator movement
    lazySusanMotor.off();
    lazySusanMotor.encoder->resetIncrement();
    bottom = lazySusanMotor.encoder->getIncrements();


          Serial.println("dr2: " + String(digitalRead(ARM_UPPER_LIMIT_SWITCH) ) );
    // turn motor in opposite direction until top limit reached
    do {
        lazySusanMotor.backward(1500);
        Serial.print("2: ");
        Serial.println(digitalRead(ARM_UPPER_LIMIT_SWITCH));
    } while (!digitalRead(ARM_UPPER_LIMIT_SWITCH));
    delay(1000);

    // initialize top of elevator movement
    lazySusanMotor.off();
    top = lazySusanMotor.encoder->getIncrements();
    lazySusanMotor.encoder->setMaxIncrement(top);

    // turn motor and reach middle of motion
    center = top / 2;
    Serial.println("center: " + String( center ) );
    while (lazySusanMotor.encoder->getIncrements() != center) {
          Serial.println("enc: " + String (lazySusanMotor.encoder->getIncrements() ));
        lazySusanMotor.forward(2000);
        Serial.print("3: ");
        Serial.println(lazySusanMotor.encoder->getIncrements());
    }
    lazySusanMotor.stop();
    Serial.println("done");

}