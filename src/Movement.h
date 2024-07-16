#include <Arduino.h>

#define MOTOR_FREQUENCY 1000

class RotaryEncoder;



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


  //RotaryEncoder* encoder;

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
  Motor(PinName PWM_pinA, PinName L_PWM_pinB);// : PWM_pinA(PWM_pinA), PWM_pinB(L_PWM_pinB);

  
  Motor(PinName PWM_pinA, PinName L_PWM_pinB, RotaryEncoder* Encoder);// : PWM_pinA(PWM_pinA), PWM_pinB(L_PWM_pinB);

  /** 
   * @brief Returns the first of 2 PWM pins
   */
  PinName getPinA();

  /**
   * @brief Returns the second of the 2 PWM pins
   */
  PinName getPinB();

  void setMotor(int motorSpeed);

  /**
   * @brief moves the motor forward at a given pwm signal
   * 
   * @param PWM_Val PWM to send to the motor 
   */
  void forward(int PWM_Val);

  /**
   * @brief moves the motor backward at a given pwm signal
   * 
   * @param PWM_Val PWM to send to the motor 
   */
  void backward(int PWM_Val);


  /**
   * @brief Stops the motor from turning. If the motor is spinning, it pulses quickly in the opposite direction
   * before sending nothing to the motors
   * 
   */
  void stop();


};


/**
 * @brief 
 * 
 */
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
  RotaryEncoder(PinName pinA, PinName pinB);

  /**
   * @brief Get the Pin A object
   * 
   * @return PinName pin attached to the first terminal
   */
  PinName getPinA();

  /**
   * @brief Get the Pin B object
   * 
   * @return PinName pin attached to the second terminal
   */
  PinName getPinB();

  /**
   * @brief Get the number of increments offset the encoder has
   * 
   * @return int number of increments from zero position
   */
  int getIncrements();

  /**
   * @brief Get the Speed measured by the encoder
   * 
   * @return int speed
   */
  double getSpeed();

  
  /**
   * @brief Updates the time of the most recent ISR call 
   * and the time between the 3 most recent calls
   * 
   * @param time current time
   */
  void updateTime(unsigned long time);

  
  /**
   * @brief updates the speed of the encoder. Must be called after the ISR call, but before the next ISR call
   * 
   */
  void updateSpeed();

  /**
   * @brief old function not needed. Still would like to keep for the time being
   * 
   * @param Aa 
   * @param Bb 
   */
  void updateEncoder(bool Aa, bool Bb);

  
  /**
   * @brief Updates the encoder object. Should be called from an ISR
   * 
   */
  void updateEncoder();

};
