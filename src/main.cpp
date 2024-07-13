#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// Inputs for local programming
#define TAB 39
#define SET_VAL 36
#define SELECT 20

// Set mode of OLED
bool programMode = 0; // low means parameters cannot be updated (competition mode)

// Parameter initialization
bool tabState = 0;
bool selectState = 0;
int setValState = 0;
int currentTab = 0; // current menu option selected

// Menu items
const int menuMax = 2; // maximum entries in each menu
bool startScreen = 0; // starts up with start screen

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


void setup() {

  // Initialize button inputs with pull-up resistors
  pinMode(TAB, INPUT_PULLUP);
  pinMode(SELECT, INPUT_PULLUP);

  if (!digitalRead(TAB) && !digitalRead(SELECT)) {
    programMode = 1;
  }

  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display_handler.display();
  delay(1000);

  if (programMode) {
    // Setup menu selection
    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
    display_handler.println("WELCOME :)");
    display_handler.println("PRESS TAB TO SCROLL THROUGH PARAMETERS!!!");
    display_handler.display();

  }
  else {
    // Set up competition mode display
    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
    display_handler.println("CAN'T TALK RN...");
    display_handler.println("TOO BUSY COOKING!!!");
    display_handler.display();
  }

}

void loop() {

}

void displayMenu(int menuNum) {

}
