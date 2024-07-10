#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_SSD1306.h>

#define BP 0
#define ESP 1

#define MASTER 1
#define SLAVE 0

#define BOARD_TYPE BP
#define STATUS SLAVE


#define RX 9
#define TX 10



#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*  analog inputs */



HardwareSerial SerialPort(1);  //if using UART1

bool toggled = false;


String received;

int loopedCount = 0;

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
  display_handler.println("Setting up...");
  display_handler.display();



SerialPort.begin(115200, SERIAL_8N1, RX, TX);

}

void loop() {
   
   


    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);

	loopedCount++;
	if(loopedCount = 1){
		//loopedCount = 0;
	    SerialPort.println("from the esp");
		display_handler.println("sent");
	}



    //display_handler.display();


    // if (SerialPort.available()) {
    //   //int fake = Serial.parseInt(); //taken out for reading strings
    // }

    if (SerialPort.available() > 0) {
	//if (true){
      received = "";
      received = SerialPort.readString();
      
      toggled = true;
    //   display_handler.clearDisplay();
    //   display_handler.setTextSize(1);
    //   display_handler.setTextColor(SSD1306_WHITE);
    //   display_handler.setCursor(0,0);
      display_handler.print("Received: ");
      display_handler.println(received);
    //   display_handler.display();
      

    } else {

    	display_handler.println("Serial port unavailable");
		display_handler.print("most recent msg: ");
		display_handler.println(received);
    }

	display_handler.display();

}
