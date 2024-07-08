#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include <Servo.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*  analog inputs */
#define POT_PIN A1

#define MOTOR_FREQUENCY 1000

/*  Motors  */


/*  Dont need these definitions anymore   */
//#define MOTOR_A PB_0
//#define MOTOR_B PB_1


/**
* assumes forwardDirection and backward direction are never both true
* PWM_pinA and PWM_pinB should not be mutable
* 
* Uses the #define motor frequency
*
 */

class Motor {

  private:
  PinName PWM_pinA;
  PinName PWM_pinB;

  int motorSpeed = 0;

  bool forwardDirection = false;
  bool backwardDirection = false;


  public:

  /**
   * @brief Construct a new Motor object with no PWM pins
   * 
   */
  Motor() = default;

  /**
   * @brief Construct a new Motor object
   * 
   * @param PWM_pinA first PWM pin controlling the motor
   * @param L_PWM_pinB Second PWM pin controlling the motor
   */
  Motor(PinName PWM_pinA, PinName L_PWM_pinB) : PWM_pinA(PWM_pinA), PWM_pinB(L_PWM_pinB) {}

  /** 
   * @brief Returns the first of 2 PWM pins
   */
  PinName getPinA(){
    return PWM_pinA;
  }

  /**
   * @brief Returns the second of the 2 PWM pins
   */
  PinName getPinB(){

    return PWM_pinB;
  }

  /**
   * @brief moves the motor forward at a given pwm signal
   * 
   * @param PWM_Val PWM to send to the motor 
   */
  void forward(int PWM_Val){
    forwardDirection = true;
    backwardDirection = false;

    this->motorSpeed = PWM_Val;

    pwm_start(PWM_pinA, MOTOR_FREQUENCY, PWM_Val, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(PWM_pinB, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);

  }

  /**
   * @brief moves the motor backward at a given pwm signal
   * 
   * @param PWM_Val PWM to send to the motor 
   */
  void backward(int PWM_Val){
    forwardDirection = false;
    backwardDirection = true;
    pwm_start(PWM_pinA, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(PWM_pinB, MOTOR_FREQUENCY, PWM_Val, RESOLUTION_12B_COMPARE_FORMAT);
  }


  /**
   * @brief Stops the motor from turning. If the motor is spinning, it pulses quickly in the opposite direction
   * before sending nothing to the motors
   * 
   */
  void stop(){
  
    if (forwardDirection){
      pwm_start(PWM_pinA, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(PWM_pinB, MOTOR_FREQUENCY, this->motorSpeed, RESOLUTION_12B_COMPARE_FORMAT);

      delay(100);
    }
    else if (backwardDirection){
      pwm_start(PWM_pinA, MOTOR_FREQUENCY, this->motorSpeed, RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start(PWM_pinB, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);

      delay(100);    
    }

    pwm_start(PWM_pinA, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(PWM_pinB, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
    
  }


};


/*  create servo object  */
Servo servo1;

/*  create motor object   */
Motor motor1(PB_0, PB_1);


void setup() {


  /*  Servo setup  */
  pinMode(PA8, OUTPUT);
  servo1.attach(PA8);


  /*  Display setup  */
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



  //old code 
  //pinMode(MOTOR_A, OUTPUT);
  //pinMode(MOTOR_B, OUTPUT);

  /*  Motor Pins  */
  pinMode(motor1.getPinA(), OUTPUT);
  pinMode(motor1.getPinB(), OUTPUT);

  /*  Other pins  */
  pinMode(POT_PIN, INPUT_ANALOG);

  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.print("Zero degrees");
  display_handler.display();

  servo1.write(0);
  delay(1000);


}

void loop() {

  int potVal = analogRead(POT_PIN);


  /*  Servo control  */

  servo1.write(210);
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("210 degrees");
  display_handler.display();
  delay(1000);

  servo1.write(90);
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("90 degrees");
  display_handler.display();
  delay(1000);

  servo1.write(0);
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("0 degrees");
  display_handler.display();
  delay(1000);



  {//DC motor Section

  int motorVal = map(potVal, 0, 1100, 0, 4096);

  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.print("Pot val: ");
  display_handler.println(potVal);

  display_handler.print("Motor val: ");
  display_handler.println(motorVal);
  display_handler.display();

  motor1.stop();
  delay(1000);

  motor1.forward(motorVal);
  delay(3000);

  motor1.stop();
  delay(1000);

  motor1.backward(motorVal);
  delay(3000);

  }



  // old code, not used anymore. kinda doesnt work

  /*

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


