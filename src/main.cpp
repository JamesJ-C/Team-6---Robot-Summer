#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



/*  Rotary  */
#define ROTARY_A PB8
#define ROTARY_B PB9


// Rotary Encoder Inputs
int counter = 0;

bool currentStateA, currentStateB;
int lastEncoded = 0b11;

void updateEncoder();
void ISRUpdateEncoder();

/*  Motors  */

#define MOTOR_FREQUENCY 1000

class RotaryEncoder {

  private:


  const int clicksPerRotation = 20; //depending on each encoder

  PinName pinA;
  PinName pinB;


  /*  for calculating direction. This value seems to make 0 position work on startup 
  otherwise there would be an offset on increment by 2 */
  int lastEncoded = 0b11; 
  int increments = 0; //net clicks offset from 0
  int previousIncrement; //net clicks at the previous time stamp
  double angularVelocity = 0; 

  unsigned long lastUpdateTime = 0; //time at which the ISR was last called
  int deltaT; //ftime between the most recent ISR calls
  
  
  public:

  /**
   * @brief Construct a new Rotary Encoder object
   * 
   * @param pinA pin attached to one terminal of the encoder
   * @param pinB pin attached to the other terminal of the encoder
   */
  RotaryEncoder(PinName pinA, PinName pinB) : pinA(pinA), pinB(pinB) {}

  /**
   * @brief Get the Pin A object
   * 
   * @return PinName pin attached to the first terminal
   */
  PinName getPinA(){
    return pinA;
  }
  /**
   * @brief Get the Pin B object
   * 
   * @return PinName pin attached to the second terminal
   */
  PinName getPinB(){
    return pinB;
  }
  /**
   * @brief Get the number of increments offset the encoder has
   * 
   * @return int number of increments from zero position
   */
  int getIncrements(){
    return increments;
  }
  /**
   * @brief Get the Speed measured by the encoder
   * 
   * @return int speed
   */
  double getSpeed(){
    this->updateSpeed();
    return angularVelocity;
  }

  
  /**
   * @brief Updates the time of the most recent ISR call 
   * and the time between the 3 most recent calls
   * 
   * @param time current time
   */
  void updateTime(unsigned long time){

    deltaT = time - this->lastUpdateTime;
    this->lastUpdateTime = time;
  }

  
  /**
   * @brief updates the speed of the encoder. Must be called after the ISR call, but before the next ISR call
   * 
   */
  void updateSpeed(){


    //THIS FUNCTION DOES NOT WORK
    angularVelocity =  ( (double) (this->increments - this->previousIncrement) / deltaT ) / (double) clicksPerRotation;

  }

  /**
   * @brief old function not needed. Still would like to keep for the time being
   * 
   * @param Aa 
   * @param Bb 
   */
  void updateEncoder(bool Aa, bool Bb){

    // bool A = digitalRead(ROTARY_A);
    // bool B = digitalRead(ROTARY_B);

    // /*	encodes 2 bit current state  */
    // int encoded = ( A << 1 ) | B;
    // /*	encodes the last states bits, concat the current states bits  */
    // int concat = ( lastEncoded << 2 ) | encoded;

    // /*	hard codes all the possibilities of encoded data  */
    // if (concat == 0b1101 || concat == 0b0100 || concat == 0b0010 || concat == 0b1011){
    //   this->increments++;
    // }
    // if (concat == 0b1110 || concat == 0b0111 || concat == 0b0001 || concat == 0b1000) {
    //   this->increments--;
    // }

    // /*	the current states bits become the next states previous bits  */
    // this->lastEncoded = encoded;


  }

  
  /**
   * @brief Updates the encoder object. Should be called from an ISR
   * 
   */
  void updateEncoder(){

    bool A = digitalRead(ROTARY_A);
    bool B = digitalRead(ROTARY_B);

    /*	encodes 2 bit current state  */
    int encoded = ( A << 1 ) | B;
    /*	encodes the last states bits, concat the current states bits  */
    int concat = ( lastEncoded << 2 ) | encoded;

    /*	hard codes all the possibilities of encoded data  */
    if (concat == 0b1101 || concat == 0b0100 || concat == 0b0010 || concat == 0b1011){
      this->increments++;
    }
    if (concat == 0b1110 || concat == 0b0111 || concat == 0b0001 || concat == 0b1000) {
      this->increments--;
    }

    /*	the current states bits become the next states previous bits  */
    this->lastEncoded = encoded;

  }

};


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


  RotaryEncoder* encoder;

  // /**
  //  * @brief Construct a new Motor object with no PWM pins
  //  * 
  //  */
  // Motor() = default;

  /**
   * @brief Construct a new Motor object
   * 
   * @param PWM_pinA first PWM pin controlling the motor
   * @param L_PWM_pinB Second PWM pin controlling the motor
   */
  Motor(PinName PWM_pinA, PinName L_PWM_pinB) : PWM_pinA(PWM_pinA), PWM_pinB(L_PWM_pinB) {}

  
  Motor(PinName PWM_pinA, PinName L_PWM_pinB, RotaryEncoder* Encoder) : PWM_pinA(PWM_pinA), PWM_pinB(L_PWM_pinB) {

    this->encoder = Encoder;

  }

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


  void setMotor(int motorSpeed){

    if (motorSpeed > 100){
      forward(motorSpeed);
    }
    else if (motorSpeed < -100){
      backward(-motorSpeed);
    }
    else {
      stop();
    }

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




RotaryEncoder encoder1(PB_8, PB_9);


#define Motor1_P1 PB_0
#define Motor1_P2 PB_1
Motor motor1(Motor1_P1, Motor1_P2, &encoder1);


/*  Pot pin  */

#define POT_PIN A1

void setup() {

  /*  Pot Pin  */
  pinMode(POT_PIN, INPUT);

  /*  Display setup  */
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display_handler.display();
  delay(2000);

	display_handler.clearDisplay();
	display_handler.setTextSize(1);
	display_handler.setTextColor(SSD1306_WHITE);
	display_handler.setCursor(0,0);
	display_handler.println("Setting up...");
	display_handler.display();


  /*  Motor Pins  */
  pinMode(motor1.getPinA(), OUTPUT);
  pinMode(motor1.getPinB(), OUTPUT);

  motor1.stop();


  /*  Encoders  */
	pinMode(ROTARY_A, INPUT);
	pinMode(ROTARY_B, INPUT);

  // pinMode(encoder1.getPinA(), INPUT);
	// pinMode(encoder1.getPinB(), INPUT);

	// Setup Serial Monitor
	Serial.begin(9600);

	attachInterrupt(digitalPinToInterrupt(ROTARY_A), ISRUpdateEncoder, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ROTARY_B), ISRUpdateEncoder, CHANGE);

  // attachInterrupt(digitalPinToInterrupt(encoder1.getPinA()), updateEncoder, CHANGE);
	// attachInterrupt(digitalPinToInterrupt(encoder1.getPinB()), updateEncoder, CHANGE);



}


int LOOP_GAIN = 1;
int P_GAIN = 30;
int I_GAIN = 0;
int D_GAIN = 10;

//use lecture slide to tune
int setVal = 32;

int measuredVal;

int error = 0;
int lastError = 0;

int max_I = 140;

int p,d,i;

int g;

void loop() {

  /*  print statements  */
  {
  /*	---------------  */

  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
	display_handler.print("Counter: ");
	display_handler.println(counter);
  display_handler.print("Obj Counter: ");
	display_handler.println(encoder1.getIncrements() );


  display_handler.print("motor.Obj Counter: ");
	display_handler.println(motor1.encoder->getIncrements() );


  display_handler.print("g: ");
	display_handler.println(g);


  display_handler.print("error: ");
	display_handler.println(error );


  // display_handler.print("Obj speed: ");
	// display_handler.println(encoder1.getSpeed() );


  display_handler.display();


  /*	---------------  */

  }

  // int readVal = analogRead(POT_PIN);

  // setVal = map(readVal, 0, 1023, -500, 500);

  measuredVal = motor1.encoder->getIncrements();

  error = setVal - measuredVal;


  p = P_GAIN * error;
  d = D_GAIN * (error - lastError);
  i = I_GAIN * error + i; //const * error + previous int value
  if (i > max_I) {i = max_I;}
  if (i < -max_I) {i = -max_I;}


  g = LOOP_GAIN * ( p + i + d );

  motor1.setMotor(g);

  lastError = error;


  //do motor code now







}


/**
 * @brief updates counter for each edge. for one click of the encoder, each output will have 2 clicks,
 * so a total of 4 increments per click. 
 * 
 */
void updateEncoder(){

	currentStateA = digitalRead(ROTARY_A);
 	currentStateB = digitalRead(ROTARY_B);

  encoder1.updateEncoder(currentStateA, currentStateB);

  encoder1.updateTime( millis() );

	/*	encodes 2 bit current state  */
	int encoded = ( currentStateA << 1 ) | currentStateB;
	/*	encodes the last states bits, concat the current states bits  */
	int concat = ( lastEncoded << 2 ) | encoded;

	/*	hard codes all the possibilities of encoded data  */
	if (concat == 0b1101 || concat == 0b0100 || concat == 0b0010 || concat == 0b1011){
		counter++;
    //encoder1.changeCount(1);
	}
	if (concat == 0b1110 || concat == 0b0111 || concat == 0b0001 || concat == 0b1000) {
		counter--;
    //encoder1.changeCount(-1);
	}
	/*	the current states bits become the next states previous bits  */
	lastEncoded = encoded;
	
}

void ISRUpdateEncoder(){

  encoder1.updateEncoder();
  encoder1.updateTime( millis() );

}