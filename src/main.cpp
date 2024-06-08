#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*  analog inputs */
#define POT_PIN A1

#define MOTOR_FREQUENCY 1000

/*  Motors  */

#define MOTOR_A PB_0
#define MOTOR_B PB_1



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


  pinMode(POT_PIN, INPUT_ANALOG);

  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly:

  int potVal = analogRead(POT_PIN);

  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.print("Pot val:");
  display_handler.println(potVal);
  
for (int i=0; i< 100000; i++){
  pwm_start(MOTOR_A, MOTOR_FREQUENCY, 3000, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_B, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);  

}

for (int i=0; i< 100000; i++){
  pwm_start(MOTOR_A, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_B, MOTOR_FREQUENCY, 3000, RESOLUTION_12B_COMPARE_FORMAT);  

}

/*

  //pwm_stop( MOTOR_A);

  pwm_start(MOTOR_A, MOTOR_FREQUENCY, map(potVal, 0, 1024, 0, 4095), RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_B, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);


  int intermediateVal = map(potVal, 0, 1024, -500, 500);
  display_handler.print("int-val:");
  display_handler.println(intermediateVal);

  if (intermediateVal < -20){
   
  pwm_start(MOTOR_A, MOTOR_FREQUENCY, -1*intermediateVal, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_B, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT); 

  display_handler.print("Motor A:");
  display_handler.println(-1*intermediateVal);

  display_handler.println("Motor B: 0");

  }

  else if (intermediateVal > 20) {

  pwm_start(MOTOR_A, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
  pwm_start(MOTOR_B, MOTOR_FREQUENCY, intermediateVal, RESOLUTION_12B_COMPARE_FORMAT);

  display_handler.println("Motor A: 0");

  display_handler.print("Motor B:");
  display_handler.println(intermediateVal);

  }

  else {
    pwm_stop(MOTOR_A);
    pwm_stop(MOTOR_B);
    display_handler.print("motors off");
  }

  display_handler.display();    

// */

}

