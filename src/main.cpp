#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

#include "RotaryEncoder.h"
#include "Motor.h"
#include "robotConstants.h"




/*  Function Declerations  */
void updateEncoder();
void ISRUpdateEncoder();
void ISRButton();



/*  Rotary Encoder valsues  */

int counter = 0;

bool currentStateA, currentStateB;
int lastEncoded = 0b11;





//NEED TO FIX THIS VARIABLE
bool buttonPressed = false;
//NEED TO FIX THIS VARIABLE


/*  PID Control Values  */
int LOOP_GAIN = 1;
int P_GAIN = 30;
int I_GAIN = 0;
int D_GAIN = 10;

//use lecture slide to tune
int setVal = 32;

int measuredVal;

int error = 0;
int lastError = 0;

int max_I = 140;

int p,d,i;

int g;


/*  Object declerations  */
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


encoder::RotaryEncoder encoder1(PB_8, PB_9);
movement::Motor motor1(Motor1_P1, Motor1_P2, &encoder1);



void setup() {


	// Setup Serial Monitor
	Serial.begin(9600);
  Serial.println("Hello" + String());

  /*  Pot Pin  */
  pinMode(POT_PIN, INPUT);

  /*  Display setup  */
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display_handler.display();
  delay(2000);

	display_handler.clearDisplay();
	display_handler.setTextSize(1);
	display_handler.setTextColor(SSD1306_WHITE);
	display_handler.setCursor(0,0);
	display_handler.println("Setting up...");
	display_handler.display();


  /*  Motor Pins  */
  pinMode(motor1.getPinA(), OUTPUT);
  pinMode(motor1.getPinB(), OUTPUT);

  motor1.off();


  /*  Encoders  */
	pinMode(ROTARY_A, INPUT);
	pinMode(ROTARY_B, INPUT);

  pinMode(BUTTON_PIN, INPUT);


  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), ISRButton, RISING);
  // pinMode(encoder1.getPinA(), INPUT);
	// pinMode(encoder1.getPinB(), INPUT);



	attachInterrupt(digitalPinToInterrupt(ROTARY_A), ISRUpdateEncoder, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ROTARY_B), ISRUpdateEncoder, CHANGE);

  // attachInterrupt(digitalPinToInterrupt(encoder1.getPinA()), updateEncoder, CHANGE);
	// attachInterrupt(digitalPinToInterrupt(encoder1.getPinB()), updateEncoder, CHANGE);



}


void loop() {

  /*  print statements  */
  {
  /*	---------------  */

  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
	display_handler.print("Counter: ");
	display_handler.println(counter);
  display_handler.print("Obj Counter: ");
	display_handler.println(encoder1.getIncrements() );


  display_handler.print("motor.Obj Counter: ");
	display_handler.println(motor1.encoder->getIncrements() );


  display_handler.print("g: ");
	display_handler.println(g);


  display_handler.print("error: ");
	display_handler.println(error );


  // display_handler.print("Obj speed: ");
	// display_handler.println(encoder1.getSpeed() );


  display_handler.display();


  /*	---------------  */

  }

  // int readVal = analogRead(POT_PIN);

  // setVal = map(readVal, 0, 1023, -500, 500);

  // measuredVal = motor1.encoder->getIncrements();

  // error = setVal - measuredVal;


  // p = P_GAIN * error;
  // d = D_GAIN * (error - lastError);
  // i = I_GAIN * error + i; //const * error + previous int value
  // if (i > max_I) {i = max_I;}
  // if (i < -max_I) {i = -max_I;}


  // g = LOOP_GAIN * ( p + i + d );

  // motor1.setMotor(g);

  // lastError = error;


  //do motor code now







}


/**
 * @brief updates counter for each edge. for one click of the encoder, each output will have 2 clicks,
 * so a total of 4 increments per click. 
 * 
 */
void updateEncoder(){

	currentStateA = digitalRead(ROTARY_A);
 	currentStateB = digitalRead(ROTARY_B);

  // encoder1.updateEncoder(currentStateA, currentStateB);

  // encoder1.updateTime( millis() );

	/*	encodes 2 bit current state  */
	int encoded = ( currentStateA << 1 ) | currentStateB;
	/*	encodes the last states bits, concat the current states bits  */
	int concat = ( lastEncoded << 2 ) | encoded;

	/*	hard codes all the possibilities of encoded data  */
	if (concat == 0b1101 || concat == 0b0100 || concat == 0b0010 || concat == 0b1011){
		counter++;
    //encoder1.changeCount(1);
	}
	if (concat == 0b1110 || concat == 0b0111 || concat == 0b0001 || concat == 0b1000) {
		counter--;
    //encoder1.changeCount(-1);
	}
	/*	the current states bits become the next states previous bits  */
	lastEncoded = encoded;
	
}

void ISRUpdateEncoder(){

  bool A = digitalRead(ROTARY_A);
  bool B = digitalRead(ROTARY_B);

  encoder1.updateEncoder(A, B);
  encoder1.updateTime( millis() );



}

void ISRButton() {


  Serial.print("inside the interrupt");
  // display_handler.clearDisplay();
	// display_handler.setTextSize(1);
	// display_handler.setTextColor(SSD1306_WHITE);
	// display_handler.setCursor(0,0);
	// display_handler.println("reset");
	// display_handler.display();
  encoder1.resetIncrement();
  //delay(100);
  buttonPressed = true;

  //motor1.buttonPressed = true;


}