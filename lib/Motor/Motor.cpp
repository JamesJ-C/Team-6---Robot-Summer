#include <Arduino.h>
#include <map>
// #include "Motor.h"
// #include "RotaryEncoder.h"


#include <Motor.h>
#include <RotaryEncoder.h>


namespace movement {

  #ifdef ESP32
    Motor::Motor(uint8_t PWM_pinA, uint8_t L_PWM_pinB, String outputName, int channel_1, int channel_2, encoder::RotaryEncoder* Encoder)
    : PWM_pinA(PWM_pinA), PWM_pinB(L_PWM_pinB){

      this->encoder = Encoder;

      pwmMap.insert({outputName + "_1", channel_1});
      pwmMap.insert({outputName + "_2", channel_2});

    }

  #endif

  /**
   * @brief Construct a new Motor object
   * 
   * @param PWM_pinA first PWM pin controlling the motor
   * @param L_PWM_pinB Second PWM pin controlling the motor
   */
  Motor::Motor(uint8_t PWM_pinA, uint8_t L_PWM_pinB) : PWM_pinA(PWM_pinA), PWM_pinB(L_PWM_pinB) {}

  // Motor::Motor(uint8_t PWM_pinA, uint8_t L_PWM_pinB, encoder::RotaryEncoder *Encoder) : PWM_pinA(PWM_pinA), PWM_pinB(L_PWM_pinB) {

  //   this->encoder = Encoder;

  // }

  /** 
   * @brief Returns the first of 2 PWM pins
   */
  uint8_t Motor::getPinA(){
    return PWM_pinA;
  }

  /**
   * @brief Returns the second of the 2 PWM pins
   */
  uint8_t Motor::getPinB(){

    return PWM_pinB;
  }

  /** ##this function is still being worked on
   * @brief 
   * 
   * @param val 
   */
  void Motor::setMotor(int val){


    if (val > 0){
      this->forward(val);
    }
    if (val < 0){
      this->backward(val);
    } else {
      this->off();
    }

    // const int maxMotorSpeed = 3500;

    // const int midMotorSpeed = 3800;

    // motorSpeed = constrain(val + midMotorSpeed, 3500, 4095);

    // this->forward(motorSpeed);

    // Serial.println("Motor " + String(PWM_pinA) + ": " + String(motorSpeed));




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


#ifdef ESP32
//func forward
  Motor::forward(int PWM_Val){
    forwardDirection = true;
    backwardDirection = false;
    this->motorSpeed = PWM_Val;

    ledcWrite( this->pwmMap.at(this->outputName + "_1") , PWM_Val );
    ledcWrite( this->pwmMap.at(this->outputName + "_2") , 0 );
  }
#endif 
#ifndef ESP32

  /**
   * @brief moves the motor forward at a given pwm signal
   * 
   * @param PWM_Val PWM to send to the motor 
   */
  void Motor::forward(int PWM_Val){
    forwardDirection = true;
    backwardDirection = false;
    this->motorSpeed = PWM_Val;

    pwm_start( (PinName) PWM_pinA, MOTOR_FREQUENCY, PWM_Val, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start( (PinName) PWM_pinB, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);

  }

#endif

#ifdef ESP32
//func backward
  Motor::backard(int PWM_Val){
    forwardDirection = false;
    backwardDirection = true;
    this->motorSpeed = PWM_Val;

    ledcWrite( this->pwmMap.at(this->outputName + "_1") , 0 );
    ledcWrite( this->pwmMap.at(this->outputName + "_2") , PWM_Val );
  }
#endif
#ifndef ESP32
  /**
   * @brief moves the motor backward at a given pwm signal
   * 
   * @param PWM_Val PWM to send to the motor 
   */
  void Motor::backward(int PWM_Val){
    forwardDirection = false;
    backwardDirection = true;
    this->motorSpeed = PWM_Val;

    pwm_start((PinName) PWM_pinA, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start((PinName) PWM_pinB, MOTOR_FREQUENCY, PWM_Val, RESOLUTION_12B_COMPARE_FORMAT);
  }

#endif

//func stop
//updated stop function
void Motor::stop() {
  delay(10);
  if (forwardDirection){
    this->backward(this->motorSpeed);

    delay(10);
  }
  else if (backwardDirection){
    this->forward(this->motorSpeed);

    delay(10);    
  }

  this->off();
}

#ifndef ESP32

  /**
   * @brief Stops the motor from turning. If the motor is spinning, it pulses quickly in the opposite direction
   * before sending nothing to the motors
   * 
   */
  // void Motor::stop(){
  
  //   if (forwardDirection){
  //     pwm_start(PWM_pinA, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
  //     pwm_start(PWM_pinB, MOTOR_FREQUENCY, this->motorSpeed, RESOLUTION_12B_COMPARE_FORMAT);

  //     delay(100);
  //   }
  //   else if (backwardDirection){
  //     pwm_start(PWM_pinA, MOTOR_FREQUENCY, this->motorSpeed, RESOLUTION_12B_COMPARE_FORMAT);
  //     pwm_start(PWM_pinB, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);

  //     delay(100);    
  //   }

  //   pwm_start(PWM_pinA, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
  //   pwm_start(PWM_pinB, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
    
  // }

#endif

#ifdef ESP32
//func off
  Motor::off(){
    forwardDirection = false;
    backwardDirection = false;
    this->motorSpeed = 0;

    ledcWrite( this->pwmMap.at(this->outputName + "_1") , 0 );
    ledcWrite( this->pwmMap.at(this->outputName + "_2") , 0 );
  }
#endif
#ifndef ESP32

  /**
   * @brief sets both pwm outputs to the motor to 0 immediately
   * 
   */
  void Motor::off(){
    forwardDirection = false;
    backwardDirection = false;
    this->motorSpeed = 0;

    pwm_start((PinName) PWM_pinA, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
    pwm_start((PinName) PWM_pinB, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
  }

#endif


  EncodedMotor::EncodedMotor(uint8_t PWM_pinA, uint8_t L_PWM_pinB, encoder::RotaryEncoder *Encoder) 
  : Motor(PWM_pinA, PWM_pinB), encoder(Encoder) {

  }


}