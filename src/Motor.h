#include <Arduino.h>


#include <Arduino.h>
#include <RotaryEncoder.h>

#define MOTOR_FREQUENCY 1000



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