#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// Inputs for local programming
#define TAB 39
#define SET_VAL 36
#define MENU 20

// Set mode of OLED
bool programMode = 0; // low means parameters cannot be updated (competition mode)

// Parameter initialization
bool tabState = 0;
bool menuState = 0;
int setValState = 0;
uint32_t tabTimer = millis(); // time tab state last changed
uint32_t menuTimer = millis(); // time menu state last changed
int buttonMinTime = 500; // minimum milliseconds between allowed state changes 

// Menu items
const int menuMax = 1; // maximum entries in each menu
String menu1[menuMax]; // entries in menu1
String menu2[menuMax]; // entries in menu2
int currentMenu = 0; // current menu selected


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void displayMenu(int menu);

void setup() {

  // Initialize button inputs with pull-up resistors
  pinMode(TAB, INPUT_PULLUP);
  pinMode(MENU, INPUT_PULLUP);

  if (!digitalRead(TAB) && !digitalRead(MENU)) {
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
    display_handler.println("MENU DISPLAY");
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

void displayMenu(int menu) {
  // Setup menu selection
    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
    display_handler.println("MENU DISPLAY");
    display_handler.display();
}