#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_SSD1306.h>

#define BP 0
#define ESP 1

#define TRANSMIT 1
#define RECEIVE 0


/*  UPDATE THESE FOR EACH BOARD   */
#define BOARD_TYPE BP
#define STATUS TRANSMIT


/*  BP pin defs  */
//#ifdef BP
#define RX PB11
#define TX PB10
//#endif

/*  ESP pin defs  * /
#ifdef ESP
#define RX 1
#define TX 3
#endif
*/

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*  analog inputs */



HardwareSerial SerialPort(USART3);
//HardwareSerial SerialPort(PB11, PB10);

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
  display_handler.println("Setting Up...");
  display_handler.display();



//SerialPort.begin(15200, SERIAL_8N1, RX, TX);
SerialPort.begin(115200);

}

void loop() {

  
    SerialPort.println("FROM BP");

    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
    display_handler.println("sent");
//    display_handler.display();

    //delay(1000);


    // if (SerialPort.available()) {
    //   int fake = Serial.parseInt();
    // }

	if (SerialPort.available() > 0) {
    //delay(10);

    //if (true) {
		toggled = true;

		String msg;
		msg = SerialPort.readString();
        display_handler.print("msg: ");
		display_handler.println(msg);
        display_handler.display();


    } 
    

    else {

        display_handler.println("no connection");

    }


	display_handler.display();

}
