#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include <Servo.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



/*  Rotary  */

#define ROTARY_A PB8
#define ROTARY_B PB9


// Rotary Encoder Inputs
#define CLK PB8
#define DT PB9

#define outputA PB8
#define outputB PB9

int counter = 0;
int currentStateCLK;
int currentStateDT;
int lastStateCLK;
String currentDir ="";

int count = 0;


bool currentStateA, currentStateB, 
	lastStateA, lastStateB;


void updateEncoder();
void updateEncoder2();


int lastEncoded = 0;

void setup() {

  /*  Display setup  */
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display_handler.display();
  delay(2000);

  // Displays "Hello world!" on the screen
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("Setting up...");
  display_handler.display();


	// Set encoder pins as inputs
	// pinMode(CLK,INPUT);
	// pinMode(DT,INPUT);

	pinMode(ROTARY_A, INPUT);
	pinMode(ROTARY_B, INPUT);

	// Setup Serial Monitor
	Serial.begin(9600);

	// Read the initial state of CLK
	//lastStateCLK = digitalRead(CLK);


	lastStateA = digitalRead(ROTARY_A);
	lastStateB = digitalRead(ROTARY_B);


  attachInterrupt(digitalPinToInterrupt(ROTARY_A), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_B), updateEncoder2, CHANGE);


}


void loop() {


	// display_handler.clearDisplay();
	// display_handler.setTextSize(1);
	// display_handler.setTextColor(SSD1306_WHITE);
	// display_handler.setCursor(0,0);
	// display_handler.print("call: ");
	// display_handler.println(count);
	// display_handler.println("AB, current");

	// display_handler.print(currentStateA);
	// display_handler.println(currentStateB);

	// display_handler.display();





/*	---------------  */

    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
	//display_handler.print("Direction: ");
	//display_handler.println(currentDir);
	display_handler.print("Counter: ");
	display_handler.println(counter);
    display_handler.display();

/*	---------------  */

}


/**
 * @brief updates counter for each edge. for one click of the encoder, each output will have 2 clicks,
 * so a total of 4 increments per click. 
 * 
 */
void updateEncoder2(){


	count++;

	currentStateA = digitalRead(ROTARY_A);
 	currentStateB = digitalRead(ROTARY_B);

	/*	encodes 2 bit current state  */
	int encoded = ( currentStateA << 1 ) | currentStateB;
	/*	encodes the last states bits, concat the current states bits  */
	int concat = ( lastEncoded << 2 ) | encoded;

	/*	hard codes all the possibilities of encoded data  */
	if (concat == 0b1101 || concat == 0b0100 || concat == 0b0010 || concat == 0b1011){
		counter++;
	}
	if (concat == 0b1110 || concat == 0b0111 || concat == 0b0001 || concat == 0b1000) {
		counter--;
	}
	/*	the current states bits become the next states previous state bits  */
	lastEncoded = encoded;
	
}


void updateEncoder(){
	// Read the current state of CLK
	currentStateCLK = digitalRead(CLK);
 	currentStateDT = digitalRead(DT);

	// If last and current state of CLK are different, then pulse occurred
	// React to only 1 state change to avoid double count
	if (currentStateCLK != lastStateCLK && currentStateCLK == 1){


		// If the DT state is different than the CLK state then
		// the encoder is rotating CCW so decrement
		if (currentStateDT != currentStateCLK) {
			counter--;
			currentDir ="CCW";
		} else if(currentStateDT == currentStateCLK) {
			// Encoder is rotating CW so increment
			counter++;
			currentDir ="CW";
		}

		else {

		currentDir = "broken";
		}

	}

	// Remember last CLK state
	lastStateCLK = currentStateCLK;
}