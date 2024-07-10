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

int counter = 0;
int currentStateCLK;
int currentStateDT;
int lastStateCLK;
String currentDir ="";


void updateEncoder();

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


	pinMode(CLK,INPUT);
	pinMode(DT,INPUT);

	// Setup Serial Monitor
	Serial.begin(9600);

	// Read the initial state of CLK
	lastStateCLK = digitalRead(CLK);

  attachInterrupt(digitalPinToInterrupt(CLK), updateEncoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(DT), updateEncoder, CHANGE);


}

void loop() {

    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
		display_handler.print("Direction: ");
		display_handler.println(currentDir);
		display_handler.print("Counter: ");
		display_handler.println(counter);
    display_handler.display();


  // //For the non-interupt code      
	// // Read the current state of CLK
	// currentStateCLK = digitalRead(CLK);

	// // If last and current state of CLK are different, then pulse occurred
	// // React to only 1 state change to avoid double count
	// if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){



  //   display_handler.clearDisplay();
  //   display_handler.setTextSize(1);
  //   display_handler.setTextColor(SSD1306_WHITE);
  //   display_handler.setCursor(0,0);
	// 	display_handler.print("In if function ");
	// 	display_handler.println();
	// 	display_handler.println("");
	// 	display_handler.println();

  //   display_handler.display();


	// 	// If the DT state is different than the CLK state then
	// 	// the encoder is rotating CCW so decrement
	// 	if (digitalRead(DT) != currentStateCLK) {
	// 		counter --;
	// 		currentDir ="CCW";
	// 	} else {
	// 		// Encoder is rotating CW so increment
	// 		counter ++;
	// 		currentDir ="CW";
	// 	}


    
	// }

  //   display_handler.clearDisplay();
  //   display_handler.setTextSize(1);
  //   display_handler.setTextColor(SSD1306_WHITE);
  //   display_handler.setCursor(0,0);
	// 	display_handler.print("Direction: ");
	// 	display_handler.println(currentDir);
	// 	display_handler.print("Counter: ");
	// 	display_handler.println(counter);

	// 	display_handler.print("currentStateCLK: ");
	// 	display_handler.println(currentStateCLK);
	// 	display_handler.print("lastStateCLK: ");
	// 	display_handler.println(lastStateCLK);

  //   display_handler.display();

	// // Remember last CLK state
	// lastStateCLK = currentStateCLK;

	// // Put in a slight delay to help debounce the reading
	// delay(10);
}


void updateEncoder(){
	// Read the current state of CLK
	currentStateCLK = digitalRead(CLK);
  delay(5);
 	currentStateDT = digitalRead(DT);

	// If last and current state of CLK are different, then pulse occurred
	// React to only 1 state change to avoid double count
	if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){



      // display_handler.clearDisplay();
      // display_handler.setTextSize(1);
      // display_handler.setTextColor(SSD1306_WHITE);
      // display_handler.setCursor(0,0);
      // display_handler.print("In if function");
      // display_handler.println();
      // display_handler.println();
      // display_handler.println();
      // display_handler.println();
      // display_handler.display();


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


    // display_handler.clearDisplay();
    // display_handler.setTextSize(1);
    // display_handler.setTextColor(SSD1306_WHITE);
    // display_handler.setCursor(0,0);
		// display_handler.print("Direction: ");
		// display_handler.println(currentDir);
		// display_handler.print("Counter: ");
		// display_handler.println(counter);

		// // display_handler.print("currentStateCLK: ");
		// // display_handler.println(currentStateCLK);
		// // display_handler.print("lastStateCLK: ");
		// // display_handler.println(lastStateCLK);

    // display_handler.display();

	}

	// Remember last CLK state
	lastStateCLK = currentStateCLK;
}