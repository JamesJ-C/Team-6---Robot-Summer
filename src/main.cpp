#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include <Servo.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



/*  Rotary  */

#define ROTARY_A A6//PB8
#define ROTARY_B A7//PB9


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


bool currentStateA, currentStateB, 
	lastStateA, lastStateB;


void updateEncoder();
void updateEncoder2();

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
  //attachInterrupt(digitalPinToInterrupt(ROTARY_B), updateEncoder2, CHANGE);


}


void loop() {


	display_handler.clearDisplay();
	display_handler.setTextSize(1);
	display_handler.setTextColor(SSD1306_WHITE);
	display_handler.setCursor(0,0);
	display_handler.print("call: ");
	display_handler.println(count);
	display_handler.println("AB, Last - current");

	display_handler.print(lastStateA);
	display_handler.println(lastStateB);

	display_handler.print(currentStateA);
	display_handler.println(currentStateB);

	display_handler.display();


	// aCurrentAnalogState = analogRead(ROTARY_A);
	// bCurrentAnalogState = analogRead(ROTARY_B);

	// if (aCurrentAnalogState > analogThreshold){
	// 	aCurrentState = HIGH;
	// }
	// else {
	// 	aCurrentState = LOW;
	// }

	// if (bCurrentAnalogState > analogThreshold) {
	// 	bCurrentState = HIGH;
	// } else {
	// 	bCurrentState = LOW;
	// }

	// if (aCurrentState != aLastState){
	// 	if (aCurrentState == HIGH && bCurrentState == LOW){
	// 		count++;
	// 		dir = "direction 1";
	// 	} else if (aCurrentState == LOW && bCurrentState == LOW) {
	// 		count--;
	// 		dir = "direction 2";
	// 	}
	// }

	// aLastState = aCurrentState;

    // display_handler.clearDisplay();
    // display_handler.setTextSize(1);
    // display_handler.setTextColor(SSD1306_WHITE);
    // display_handler.setCursor(0,0);
	// display_handler.print("Direction: ");
	// display_handler.println(dir);
	// display_handler.print("Count: ");
	// display_handler.println(count);

	// display_handler.print("aCurrAnalog: ");
	// display_handler.println(aCurrentAnalogState);
	// display_handler.print("bCurrentAnalog: ");
	// display_handler.println(bCurrentAnalogState);


    // display_handler.display();


/*	---------------  */

    // display_handler.clearDisplay();
    // display_handler.setTextSize(1);
    // display_handler.setTextColor(SSD1306_WHITE);
    // display_handler.setCursor(0,0);
	// display_handler.print("Direction: ");
	// display_handler.println(currentDir);
	// display_handler.print("Counter: ");
	// display_handler.println(counter);
    // display_handler.display();

/*	---------------  */

// 	 aState = digitalRead(outputA); // Reads the "current" state of the outputA
//    // If the previous and the current state of the outputA are different, that means a Pulse has occured
//    if (aState != aLastState && aState == 1){     
//      // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
//      if (digitalRead(outputB) != aState) { 
//        counter ++;
//      } else {
//        counter --;
//      }
// 	display_handler.clearDisplay();
// 	display_handler.setTextSize(1);
// 	display_handler.setTextColor(SSD1306_WHITE);
// 	display_handler.setCursor(0,0);
// 	display_handler.println("Position: ");
// 	display_handler.println(counter);
// 	display_handler.display();
//    } 
//    aLastState = aState; // Updates the previous state of the outputA with the current state



/*	---------------  */

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


void updateEncoder2(){


	count++;

	currentStateA = digitalRead(ROTARY_A);
 	currentStateB = digitalRead(ROTARY_B);

	lastStateA = currentStateA;
	lastStateB = currentStateB;
	
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