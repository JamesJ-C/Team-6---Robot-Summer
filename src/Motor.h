#pragma once
#include <Arduino.h>


#include "RotaryEncoder.h"

#define MOTOR_FREQUENCY 1000


/**
* assumes forwardDirection and backward direction are never both true
* PWM_pinA and PWM_pinB should not be mutable
* 
* Uses the #define motor frequency
*
 */

namespace movement {

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
        Motor(PinName PWM_pinA, PinName L_PWM_pinB): PWM_pinA(PWM_pinA), PWM_pinB(L_PWM_pinB) {}

        
        Motor(PinName PWM_pinA, PinName L_PWM_pinB, encoder::RotaryEncoder* Encoder) : PWM_pinA(PWM_pinA), PWM_pinB(L_PWM_pinB) {}

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


        void setMotor(int motorSpeed){

            if (motorSpeed > 100){
                Motor::forward(motorSpeed);
            }
            else if (motorSpeed < -100){
                Motor::backward(-motorSpeed);
            }
            else {
                Motor::stop();
            }
        }

    };

}