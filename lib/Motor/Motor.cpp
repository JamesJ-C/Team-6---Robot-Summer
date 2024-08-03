#include <Arduino.h>
#include <map>
// #include "Motor.h"
// #include "RotaryEncoder.h"


#include <Motor.h>
#include <RotaryEncoder.h>


namespace movement {

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

  /**
   * @brief if val > 0, motor will go forward, 
   * if val < 0, motor will go backward, 
   * if val = 0, motor will be off 
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


  /**
   * @brief moves the motor forward at a given pwm signal
   * 
   * @param PWM_Val PWM to send to the motor 
   */
  void Motor::forward(int PWM_Val){
    forwardDirection = true;
    backwardDirection = false;
    this->motorSpeed = PWM_Val;
      
    #ifndef ESP32
      pwm_start( (PinName) PWM_pinA, MOTOR_FREQUENCY, PWM_Val, RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start( (PinName) PWM_pinB, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
    #endif 
    #ifdef ESP32
      analogWrite(this->PWM_pinA, PWM_Val);
      //analogWrite(this->PWM_pinA, 0);
      analogWrite(this->PWM_pinB, 0);
      // Serial.println("ESP stuff");
      // pwmForward(this->PWM_pinA,this->PWM_pinB, PWM_Val );
      Serial.println("fwd");
    #endif

  }


void Motor::espForward(int PWM_Val){
    forwardDirection = true;
    backwardDirection = false;
    this->motorSpeed = PWM_Val;

    analogWrite(this->PWM_pinA, PWM_Val);
    analogWrite(25, 0);

  }


  /**
   * @brief moves the motor backward at a given pwm signal
   * 
   * @param PWM_Val PWM to send to the motor 
   */
  void Motor::backward(int PWM_Val){
    forwardDirection = false;
    backwardDirection = true;
    this->motorSpeed = PWM_Val;

    #ifndef ESP32
      pwm_start( (PinName) PWM_pinA, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start( (PinName) PWM_pinB, MOTOR_FREQUENCY, PWM_Val, RESOLUTION_12B_COMPARE_FORMAT);
    #endif 
    #ifdef ESP32
      analogWrite(this->PWM_pinA, 0);
      // analogWrite(this->PWM_pinB, 0);
      analogWrite(this->PWM_pinB, PWM_Val);
      Serial.println("bckwd");

      // pwmBackward(this->PWM_pinA,this->PWM_pinB, PWM_Val );
    #endif
  }

    void Motor::espBackward(int PWM_Val){
    forwardDirection = false;
    backwardDirection = true;
    this->motorSpeed = PWM_Val;
    
    analogWrite(this->PWM_pinA, 0);
  //  analogWrite(this->PWM_pinB, PWM_Val);
    analogWrite(25, PWM_Val);
  }


//func stop
//updated stop function
void Motor::stop() {
  this->off();
  delay(10);
  if (forwardDirection){
    this->backward(this->motorSpeed);
    delay(10);
  } else if (backwardDirection){
    this->forward(this->motorSpeed);
    delay(10);    
  }
  this->off();
}

  /**
   * @brief sets both pwm outputs to the motor to 0 immediately
   * 
   */
  void Motor::off(){
    forwardDirection = false;
    backwardDirection = false;
    this->motorSpeed = 0;

    #ifndef ESP32
      pwm_start((PinName) PWM_pinA, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
      pwm_start((PinName) PWM_pinB, MOTOR_FREQUENCY, 0, RESOLUTION_12B_COMPARE_FORMAT);
    #endif 
    #ifdef ESP32
      analogWrite(this->PWM_pinA, 0);
      analogWrite(this->PWM_pinB, 0);
      Serial.println("off");
    #endif
  }




  EncodedMotor::EncodedMotor(uint8_t PWM_pinA, uint8_t PWM_pinB, encoder::RotaryEncoder *Encoder) 
  : Motor(PWM_pinA, PWM_pinB), encoder(Encoder) {

    // Serial.println("cons - 1: " + String(PWM_pinA) + ". 2: " + String (PWM_pinB));
    

  }


}