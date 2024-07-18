
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <vector>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define REFLECTANCE_0 PA_2 
#define REFLECTANCE_1 PA_3


void setup() {
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 

  Serial.begin(9600);

  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display_handler.display();
  delay(2000);

  // Displays "Hello world!" on the screen
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("Setup...");
  display_handler.display();

  
  // Set up reflectance inputs
  pinMode(REFLECTANCE_0, INPUT);
  pinMode(REFLECTANCE_1, INPUT);


}

void loop() {


	int reflectanceReading0 = analogRead(REFLECTANCE_0);
	int reflectanceReading1 = analogRead(REFLECTANCE_1);


	// Displays "Hello world!" on the screen
	// display_handler.clearDisplay();
	// display_handler.setTextSize(1);
	// display_handler.setTextColor(SSD1306_WHITE);
	// display_handler.setCursor(0,0);
	// display_handler.println("reflectance 0: ");
	// display_handler.println(reflectanceReading0);
	// display_handler.println("reflectance 1: ");
	// display_handler.println(reflectanceReading1);
	// display_handler.display();


	Serial.print("reflectance 0: ");
	Serial.println(reflectanceReading0);
	Serial.print("reflectance 1: ");
	Serial.println(reflectanceReading1);


	//delay(100);



}

