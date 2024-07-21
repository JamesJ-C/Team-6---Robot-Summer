#ifndef MOTOR_H 
#define MOTOR_H 

#include <Arduino.h>
#include <map>
// #include "RotaryEncoder.h"


// #include <Motor.h>
#include <RotaryEncoder.h>

namespace movement {

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

    #ifndef ESP32
        //create map
        std::map<String, int> pwmMap;
        String outputName;
    #endif


    public:

    //need to update this variables access. 
    bool buttonPressed = false;


    encoder::RotaryEncoder* encoder;


    #ifndef ESP32

        Motor(PinName PWM_pinA, PinName L_PWM_pinB, String outputName, int channel_1, int channel_2, encoder::RotaryEncoder* Encoder);

    #endif




    /**
     * @brief Construct a new Motor object
     * 
     * @param PWM_pinA first PWM pin controlling the motor
     * @param L_PWM_pinB Second PWM pin controlling the motor
     */
    Motor(PinName PWM_pinA, PinName L_PWM_pinB);

    /**
     * @brief Construct a new Motor object
     * 
     * @param PWM_pinA first PWM pin controlling the motor
     * @param L_PWM_pinB Second PWM pin controlling the motor
     * @param Encoder encoder object to attach to the motor
     */
    Motor(PinName PWM_pinA, PinName L_PWM_pinB, encoder::RotaryEncoder* Encoder);

    /** 
     * @brief Returns the first of 2 PWM pins
     */
    PinName getPinA();

    /**
     * @brief Returns the second of the 2 PWM pins
     */
    PinName getPinB();

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

    /**
     * @brief immediately sets the motor speed to 0 without any reverse direction pulse
     * 
     */
    void off();

    /**
     * @brief sets up the encoder by going to the limits of the switches 
     * and saves the difference between the values. Sets one of the limit switches to 0 increments 
     * 
     */
    void setupEncoder ();


    };

}




#endif