#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Motor.h>
#include <RobotSubSystem.h>

namespace robot {


        /**
         * @brief Construct a new Robot Sub System object
         * 
         * @param motor motor driving the movement of the system
         * @param limit1 first limit switch attatched to movement
         * @param limit2 second limit switched attatched to movement
         */
        RobotSubSystem::RobotSubSystem (uint8_t limit1, uint8_t limit2, movement::Motor *motor) { 
            this->limit1 = limit1;
            this->limit2 = limit2;
            this->motor = motor;
        }

        /**
         * @brief Returns the first limit switch pin
         */
        uint8_t RobotSubSystem::getLimit1() {
            return limit1;
        }

        /**
         * @brief Returns the second limit switch pin
         */
        uint8_t RobotSubSystem::getLimit2() {
            return limit2;
        }


        /**
         * @brief performs full movement sweep and initializes rotary encoder values 
         */
        void RobotSubSystem::localize() {

            int bottom;
            int top;
            int center;

            // turn motor until elevator reaches first limit
            do {
                this->motor->forward(1500);
            } while (!digitalRead(limit1));
            
            // initialize first limit of motion
            this->motor->off();
            this->motor->encoder->resetIncrement();
            bottom = this->motor->encoder->getIncrements();

            // turn motor in opposite direction until second limit reached
            do {
                this->motor->backward(1500);
            } while (!digitalRead(limit2));

            // initialize second limit of motion
            this->motor->off();
            top = this->motor->encoder->getIncrements();
            this->motor->encoder->setMaxIncrement(top);

            // turn motor and reach center of motion
            center = top / 2;
            while (this->motor->encoder->getIncrements() != center) {
                this->motor->forward(2000);
            }
            this->motor->stop();
        }


        /**
         * @brief rotates motor until encoder reaches the specified value
         * 
         * @param rotaryVal specific rotary encoder value to move to
         */
        void RobotSubSystem::moveToValue(int rotaryVal) {

            // get current rotary value and compare to target value
            int currentRotaryVal = this->motor->encoder->getIncrements();
            while (currentRotaryVal != rotaryVal) {
                updatePID(currentRotaryVal, rotaryVal);
                int currentRotaryVal = this->motor->encoder->getIncrements();
            }
        }


        /**
         * @brief update PID transfer function and send value to motors
         * 
         */
        void RobotSubSystem::updatePID(int current, int target) {
            
            // calculate difference between current encoder value and target encoder value
            int error = target - current;

            // calculate PID transfer function to send to motor
            double motor_p = pGain * error;
            double motor_d = dGain * (error - lastError);
            double motor_i = iGain * error + iTerm;
            if (iTerm > maxI) {iTerm = maxI;}
            if (iTerm < -maxI) {iTerm = -maxI;}

            transfer = loopGain * (motor_p + motor_d + motor_i);
            lastError = error;

            // set motor value based on calculated PID value
            this->motor->setMotor(transfer);
        }

}