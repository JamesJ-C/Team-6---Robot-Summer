#include <Arduino.h>

#include <Wire.h>

#include <HardwareSerial.h>

#include <Adafruit_SSD1306.h>

#define MASTER 1
#define SLAVE 0

#define STATUS MASTER


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*  analog inputs */



HardwareSerial SerialPort(1);  //if using UART1

//char number = '';

bool toggled = false;

void setup() {

  

  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 
  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display_handler.display();
  delay(2000);

  // Displays "Hello world!" on the screen
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("Setting up...vroom");
  display_handler.display();


  


SerialPort.begin(15200, SERIAL_8N1, 1, 3);

}

void loop() {


//Master Code /*
if (STATUS == MASTER){
  
  SerialPort.print(1);

  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("HIGH");
  display_handler.display();

  delay(1000);

  SerialPort.print(0);

  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("Low");
  display_handler.display();

  delay(1000);
}
// */

//Slave Code /*

if (STATUS == SLAVE) {

  //if (SerialPort.available() > 0) {

  if (true) {
    toggled = true;

    char number = SerialPort.read();

    if (number == '0') {

      display_handler.clearDisplay();
      display_handler.setTextSize(1);
      display_handler.setTextColor(SSD1306_WHITE);
      display_handler.setCursor(0,0);
      display_handler.println("Low");
      display_handler.display();
    }
    if (number == '1') {

      display_handler.clearDisplay();
      display_handler.setTextSize(1);
      display_handler.setTextColor(SSD1306_WHITE);
      display_handler.setCursor(0,0);
      display_handler.println("HIGH");
      display_handler.display();

    }

  } 

  else {

      if (toggled) {

      display_handler.clearDisplay();
      display_handler.setTextSize(1);
      display_handler.setTextColor(SSD1306_WHITE);
      display_handler.setCursor(0,0);
      display_handler.println("Toggled");
      display_handler.display();

      } else{


      display_handler.clearDisplay();
      display_handler.setTextSize(1);
      display_handler.setTextColor(SSD1306_WHITE);
      display_handler.setCursor(0,0);
      display_handler.println(SerialPort.available());
      display_handler.display();
      }
  }

}
// */

/*
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("done");
  display_handler.display();

*/


  
}

