#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_SSD1306.h>

#define BP 0
#define ESP 1

#define MASTER 1
#define SLAVE 0

#define BOARD_TYPE BP
#define STATUS MASTER


/*  BP pin defs  */
#ifdef BOARD_TYPE BP
#define RX PB_11
#define TX PB_10
#endif

/*  ESP pin defs  */
#ifdef BOARD_TYPE ESP
#define RX 1
#define TX 3
#endif


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



SerialPort.begin(15200, SERIAL_8N1, RX, TX);

}

void loop() {


//Master Code /*
  if (STATUS == MASTER){
    
    SerialPort.println("Flip the burger!");

    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
    display_handler.println("sent");
    display_handler.display();

    delay(1000);


  }
// */

//Slave Code /*

  if (STATUS == SLAVE) {


    if (SerialPort.available()) {
      //int fake = Serial.parseInt();
    }

    if (SerialPort.available() > 0) {
      
      String received = "";
      received = SerialPort.readString();
      
      toggled = true;
      display_handler.clearDisplay();
      display_handler.setTextSize(1);
      display_handler.setTextColor(SSD1306_WHITE);
      display_handler.setCursor(0,0);
      display_handler.print("Received: ");
      display_handler.println(received);
      display_handler.display();
      

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

}
