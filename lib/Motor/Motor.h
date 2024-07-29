#ifndef MOTOR_H 
#define MOTOR_H 

#include <Arduino.h>
#include <map>
// #include "RotaryEncoder.h"

#include <robotConstants.h>

// #include <Motor.h>
#include <RotaryEncoder.h>

#ifdef ESP32
#include <bpConstants.h>
#endif

#ifndef ESP32
#include <espConstants.h>
#endif

namespace movement {


    /**
    * assumes forwardDirection and backward direction are never both true
    * PWM_pinA and PWM_pinB should not be mutable
    * 
    * Uses the #define motor frequency
    *
    */

    class Motor {

    protected:
    uint8_t PWM_pinA;
    uint8_t PWM_pinB;

    int motorSpeed = 0;

    bool forwardDirection = false;
    bool backwardDirection = false;

    #ifdef ESP32
        //create map
        std::map<String, int> pwmMap;
        String outputName;
    #endif


    public:

    /**
     * @brief Construct a new Motor object
     * 
     * @param PWM_pinA first PWM pin controlling the motor
     * @param L_PWM_pinB Second PWM pin controlling the motor
     */
    Motor(uint8_t PWM_pinA, uint8_t L_PWM_pinB);

    // /**
    //  * @brief Construct a new Motor object
    //  * 
    //  * @param PWM_pinA first PWM pin controlling the motor
    //  * @param L_PWM_pinB Second PWM pin controlling the motor
    //  * @param Encoder encoder object to attach to the motor
    //  */
    // Motor(uint8_t PWM_pinA, uint8_t L_PWM_pinB, encoder::RotaryEncoder* Encoder);

    /** 
     * @brief Returns the first of 2 PWM pins
     */
    uint8_t getPinA();

    /**
     * @brief Returns the second of the 2 PWM pins
     */
    uint8_t getPinB();

    /**
     * @brief Set the speed of the motor.
     * 
     * @param motorSpeed motor speed, can be positive or negative, abs < 4096
     */
    void setMotor(int motorSpeed);

    /**
     * @brief moves the motor forward at a given pwm signal
     * 
     * @param PWM_Val PWM to send to the motor 
     */
    void forward(int PWM_Val);

    void espForward(int PWM_Val);

    /**
     * @brief moves the motor backward at a given pwm signal
     * 
     * @param PWM_Val PWM to send to the motor 
     */
    void backward(int PWM_Val);

    void espBackward(int PWM_Val);

    /**
     * @brief Stops the motor from turning. If the motor is spinning, it pulses quickly in the opposite direction
     * before sending nothing to the motors
     * 
     */
    void stop();

    /**
     * @brief immediately sets the motor speed to 0 without any reverse direction pulse
     * 
     */
    void off();

    static void pwmForward(int pin1, int pin2, int speed){
        analogWrite(pin1, speed);
        analogWrite(pin2, 0);
        Serial.println("1: " + String(pin1) + ". 2: " + String (pin2));
    }
    static void pwmBackward(int pin1, int pin2, int speed){
        analogWrite(pin2, speed);
        analogWrite(pin1, 0);
    }


    };


    class EncodedMotor : public Motor {
        public:
        encoder::RotaryEncoder *encoder;

        /**
         * @brief Construct a new Motor object
         * 
         * @param PWM_pinA first PWM pin controlling the motor
         * @param L_PWM_pinB Second PWM pin controlling the motor
         * @param Encoder encoder object to attach to the motor
         */
        EncodedMotor(uint8_t PWM_pinA, uint8_t L_PWM_pinB, encoder::RotaryEncoder* Encoder);


    };

}




#endif