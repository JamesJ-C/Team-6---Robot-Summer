#include <Arduino.h>
#include <Wire.h>
//#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include <SPI.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
//Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


/*  analog inputs */
#define POT_PIN A1

#define MOTOR_FREQUENCY 1000

/*  Motors  */

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

/*  create motor objects   */
#define Motor1_P1 PB_0
#define Motor1_P2 PB_1
Motor motor1(Motor1_P1, Motor1_P2);

#define Motor2_P1 PB_6
#define Motor2_P2 PB_7
Motor motor2(Motor2_P1, Motor2_P2);


#define TAPE_THRESHOLD 500
#define TAPE_SENSOR_PIN PA0


void setup() {


  /*  Servo setup  */
//   pinMode(PA8, OUTPUT);
//   servo1.attach(PA8);


  /*  Display setup  */
//   display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 

//   display_handler.display();
//   delay(2000);

//   // Displays "Hello world!" on the screen
//   display_handler.clearDisplay();
//   display_handler.setTextSize(1);
//   display_handler.setTextColor(SSD1306_WHITE);
//   display_handler.setCursor(0,0);
//   display_handler.println("Setting up...");
//   display_handler.display();


  /*  Motor Pins  */
  	pinMode(motor1.getPinA(), OUTPUT);
  	pinMode(motor1.getPinB(), OUTPUT);

	pinMode(motor2.getPinA(), OUTPUT);
  	pinMode(motor2.getPinB(), OUTPUT);

  /*  Other pins  */
  	pinMode(POT_PIN, INPUT_ANALOG);

//   display_handler.clearDisplay();
//   display_handler.setTextSize(1);
//   display_handler.setTextColor(SSD1306_WHITE);
//   display_handler.setCursor(0,0);
//   display_handler.print("150 degrees");
//   display_handler.display();

//   servo1.write(150);
//   delay(1000);

	motor1.stop();
	motor2.stop();
	delay(1500);


}


void loop() {
  	const int driveSpeed = 2000;

	//if (analogRead(TAPE_SENSOR_PIN) >= TAPE_THRESHOLD){

	// 	display_handler.clearDisplay();
	// 	display_handler.setTextSize(1);
	// 	display_handler.setTextColor(SSD1306_WHITE);
	// 	display_handler.setCursor(0,0);
	// 	display_handler.println("Driving...");
	// 	display_handler.display();

	// 	motor1.forward(driveSpeed);
	// 	motor2.forward(driveSpeed);
	// 	delay(1000);

	// //} else{


	// 	display_handler.clearDisplay();
	// 	display_handler.setTextSize(1);
	// 	display_handler.setTextColor(SSD1306_WHITE);
	// 	display_handler.setCursor(0,0);
	// 	display_handler.println("Driving...");
	// 	display_handler.display();

		motor1.stop();
		//motor2.stop();
		delay(200);
	//}













}
