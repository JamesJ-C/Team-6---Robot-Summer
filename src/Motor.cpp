#include <Arduino.h>
#include "Motor.h"
#include "RotaryEncoder.h"


namespace movement {


  /**
   * @brief Construct a new Motor object
   * 
   * @param PWM_pinA first PWM pin controlling the motor
   * @param L_PWM_pinB Second PWM pin controlling the motor
   */
  Motor::Motor(PinName PWM_pinA, PinName L_PWM_pinB) : PWM_pinA(PWM_pinA), PWM_pinB(L_PWM_pinB) {}

  Motor::Motor(PinName PWM_pinA, PinName L_PWM_pinB, encoder::RotaryEncoder* Encoder) : PWM_pinA(PWM_pinA), PWM_pinB(L_PWM_pinB) {

    this->encoder = Encoder;

  }

  /** 
   * @brief Returns the first of 2 PWM pins
   */
  PinName Motor::getPinA(){
    return PWM_pinA;
  }

  /**
   * @brief Returns the second of the 2 PWM pins
   */
  PinName Motor::getPinB(){

    return PWM_pinB;
  }

  void Motor::setMotor(int val){


    const int maxMotorSpeed = 3500;

    const int midMotorSpeed = 3800;

    motorSpeed = constrain(val + midMotorSpeed, 3500, 4095);

    this->forward(motorSpeed);

    Serial.println("Motor " + String(PWM_pinA) + ": " + String(motorSpeed));




    // if (motorSpeed > 100){
    //   forward(motorSpeed);
    // }
    // else if (motorSpeed < -100){
    //   backward(-motorSpeed);
    // }
    // else {
    //   stop();
    // }

  }

  /**
   * @brief moves the motor forward at a given pwm signal
   * 
   * @param PWM_Val PWM to send to the motor 
   */
  void Motor::forward(int PWM_Val){
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
  void Motor::backward(int PWM_Val){
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
  void Motor::stop(){
  
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

  void Motor::off(){
    pwm_start(PWM_pinA, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start(PWM_pinB, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
  }

  /**
   * @brief sets up the encoder by going to the limits of the switches 
   * and saves the difference between the values. Sets one of the limit switches to 0 increments 
   * 
   */
  void Motor::setupEncoder (){

    int firstStop;
    int secondStop;

    //turn all the way one way until switch
    this->backward(3000);
    while (true) {
      if (buttonPressed){
        this->off();
        firstStop = this->encoder->getIncrements();
        this->encoder->resetIncrement();
        break;
      }
    }
    this->forward(3000);
    while (true) {
      if (buttonPressed){
        this->off();
        secondStop = this->encoder->getIncrements();
        //this->encoder->resetIncrement();
        break;
      }
    }

    this->encoder->setMaxIncrement(secondStop);

  }

}