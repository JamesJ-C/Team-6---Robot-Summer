#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

#include <HardwareSerial.h>

#include "RotaryEncoder.h"
#include "Motor.h"
#include "robotConstants.h"


//test code to try smth
String board;
int boardNum = -1;

#ifdef ESP32

board = "esp32";
boardNum = 1;

#endif

#ifdef BLUEPILL_F103C8

board = "BP";
boardNum = 2;

#define G 3

#endif

#ifdef ARDUINO_ARCH_STM32

#define G 1

#endif
// end test

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
double LOOP_GAIN = 1.0 / 1.0;
double P_GAIN = 1.4;//1.6;//1.4 goes very slowl
int I_GAIN = 0;
int D_GAIN = 0;

//use lecture slide to tune
int setVal = 32;

int measuredVal;

double error = 0.0;
double lastError = 0.0;

int max_I = 140;

double p,d,i;

double g;


/*  Object declerations  */

encoder::RotaryEncoder encoder1(PB_8, PB_9);
movement::Motor MotorL(MotorL_P1, MotorL_P2);//, &encoder1);
movement::Motor MotorR(MotorR_P1, MotorR_P2);


void setup() {

  SerialPort.begin(115200);



	// Setup Serial Monitor
	Serial.begin(9600);
  Serial.println("Hello" + String(BOARD_NAME));

  Serial.println(boardNum);

  /*  Pot Pin  */
  pinMode(POT_PIN, INPUT);


  /*  Motor Pins  */
  pinMode(MotorL.getPinA(), OUTPUT);
  pinMode(MotorL.getPinB(), OUTPUT);

  pinMode(MotorR.getPinA(), OUTPUT);
  pinMode(MotorR.getPinB(), OUTPUT);

  MotorL.off();
  MotorR.off();


  /*  Encoders  */
	pinMode(ROTARY_A, INPUT);
	pinMode(ROTARY_B, INPUT);
  // pinMode(encoder1.getPinA(), INPUT);
	// pinMode(encoder1.getPinB(), INPUT);

  pinMode(BUTTON_PIN, INPUT);


  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), ISRButton, RISING);

	attachInterrupt(digitalPinToInterrupt(ROTARY_A), ISRUpdateEncoder, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ROTARY_B), ISRUpdateEncoder, CHANGE);

}


void loop() {

//   /*  print statements  */
//   {
//   /*	---------------  */

//   display_handler.clearDisplay();
//   display_handler.setTextSize(1);
//   display_handler.setTextColor(SSD1306_WHITE);
//   display_handler.setCursor(0,0);
// // 	// display_handler.print("Counter: ");
// // 	// display_handler.println(counter);
// //   // display_handler.print("Obj Counter: ");
// 	display_handler.println(encoder1.getIncrements() );


//   display_handler.print("motor.Obj Counter: ");
// 	display_handler.println(MotorL.encoder->getIncrements() );


// //   // display_handler.print("g: ");
// // 	// display_handler.println(g);


// //   // display_handler.print("error: ");
// // 	// display_handler.println(error );


// //   // // display_handler.print("Obj speed: ");
// // 	// // display_handler.println(encoder1.getSpeed() );


//   display_handler.display();


//   /*	---------------  */

//   }

  // int readVal = analogRead(POT_PIN);

  // setVal = map(readVal, 0, 1023, -500, 500);

  // measuredVal = MotorL.encoder->getIncrements();

  // error = setVal - measuredVal;

  error = analogRead(FRONT_TAPE_SENSOR_1) - analogRead(FRONT_TAPE_SENSOR_2);

  // Serial.println( "tape 1: " + String( analogRead(FRONT_TAPE_SENSOR_1) ));
  // Serial.println( "tape 2: " + String( analogRead(FRONT_TAPE_SENSOR_2 ) ));


  SerialPort.println( "tape 1: " + String( analogRead(FRONT_TAPE_SENSOR_1) ));
  SerialPort.println( "tape 2: " + String( analogRead(FRONT_TAPE_SENSOR_2 ) ));


  p = P_GAIN * error;
  d = D_GAIN * (error - lastError);
  i = I_GAIN * error + i; //const * error + previous int value
  if (i > max_I) {i = max_I;}
  if (i < -max_I) {i = -max_I;}


  const int speedGain = 3.0;

  g = LOOP_GAIN * (double) ( p + i + d ) * speedGain;

  // Serial.println("Transfer function: " + String(g));

  // if (g > 0)
  //   Serial.println("dir 1");
  // else if (g < 0)
  //   Serial.println("dir 2");
  // else 
  //   Serial.print("forward");

  // MotorL.setMotor(g);
  // MotorR.setMotor(-1 * g);

  const int midMotorSpeed = 3800;


  // if (g > 20){
  //   MotorL.forward( 1.0 * (3400) );
  //   MotorR.forward(  ( 3900) );
  // }
  // else if (g < 20) {
  //   MotorL.forward( 1.0 * (3900) );
  //   MotorR.forward(  ( 3400) );
  // }
  // else {
  //   MotorL.forward( 3500 );
  //   MotorR.forward( 3500 );
  // }



  // MotorL.forward( 1.0 * (midMotorSpeed - 0.5 * g) );
  // MotorR.forward( ( ( midMotorSpeed + g) * 0.6/3*2.1) );

  // SerialPort.println("m1: " + String( midMotorSpeed - g) );
  // SerialPort.println("m2: " + String( midMotorSpeed + g) );

  MotorL.forward( 3400 );
  MotorR.forward(3400);

  lastError = error;


  //do motor code now



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

  //MotorL.buttonPressed = true;

}