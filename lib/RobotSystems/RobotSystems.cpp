//#include <Arduino.h>
#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Motor.h>
#include <RobotSystems.h>
#include <HardwareSerial.h>


namespace robot {


        /**
         * @brief Construct a new Robot Sub System object
         * 
         * @param motor motor driving the movement of the system
         * @param limit1 first limit switch attatched to movement
         * @param limit2 second limit switched attatched to movement
         */
        RobotSubSystem::RobotSubSystem (uint8_t limit1, uint8_t limit2, movement::EncodedMotor *motor) { 
            this->limit1 = limit1;
            if (limit2 == 255){
                singleLimitSwitch = true;    
            }
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
        void RobotSubSystem::localize(const int motorSpeedForward, const int motorSpeedBackward) {

            int bottom;
            int top;
            int center;

            // turn motor until elevator reaches first limit
            
            this->motor->forward(motorSpeedForward);
            do {
                
            } while (!digitalRead(limit2)); //while (!secondSwitchHit); 
            //Serial.println("after first loop");
            
            // initialize first limit of motion
            this->motor->off();
            this->motor->encoder->resetIncrement();
            bottom = this->motor->encoder->getIncrements();

            // if (singleLimitSwitch){
            //     return;
            // }

            // turn motor in opposite direction until second limit reached
                this->motor->backward(motorSpeedBackward);
            do {
                //Serial.println("bckwd");

            } while (!digitalRead(limit1)); //while (!firstSwitchHit);

            //Serial.println("after 2nd loop");

            // initialize second limit of motion
            this->motor->off();
            top = this->motor->encoder->getIncrements();
            this->motor->encoder->setMaxIncrement(top);

            // turn motor and reach center of motion
            center = top / 2;
            while (this->motor->encoder->getIncrements() != center) {
                break;
                this->motor->forward(motorSpeedForward);

            }
            Serial.println(center);
            this->motor->stop();
            //Serial.println("after last loop");
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
                break;
                updatePID(rotaryVal);
                int currentRotaryVal = this->motor->encoder->getIncrements();
            }
        }


        /**
         * @brief update PID transfer function and send value to motors
         * 
         * @returns current error computed
         */
        int RobotSubSystem::updatePID(int target) {

            // calculate difference between current encoder value and target encoder value
            //int error = target - current;
            double error = (double) target - this->motor->encoder->getIncrements();
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
            //return (int) error;
            Serial.println(error);
            Serial.println(transfer);
            return (int) transfer;
        }



        DrivePID::DrivePID(uint8_t forwardTape1, uint8_t forwardTape2, uint8_t backwardTape1, uint8_t backwardTape2,
        movement::Motor *motorL, movement::Motor *motorR) 
        : forwardTapeSensorPin1(forwardTape1), forwardTapeSensorPin2(forwardTape2), backwardTapeSensorPin1(backwardTape1), 
        backwardTapeSensorPin2(backwardTape2), driveMotorL(motorL), driveMotorR(motorR) {}


        void DrivePID::updateForwardDrivePID() {
            //int  = 9;
            int forwardError = analogRead(this->forwardTapeSensorPin1) - analogRead(this->forwardTapeSensorPin2);

            // Serial.println("tp1: " + String( analogRead(this-> forwardTapeSensorPin1)) );
            // Serial.println("tp2: " + String( analogRead(this-> forwardTapeSensorPin2)) );
            // Serial.print("error: " );
            //Serial.println( forwardError );

            forward_p = FORWARD_P_GAIN * forwardError;
            forward_d = FORWARD_D_GAIN * (forwardError - forwardLastError);
            forward_i = FORWARD_I_GAIN * forwardError + forward_i; //const * error + previous int value

            if (forward_i > MAX_FORWARD_I) {forward_i = MAX_FORWARD_I;}
            if (forward_i < -MAX_FORWARD_I) {forward_i = -MAX_FORWARD_I;}

            forward_g = FORWARD_LOOP_GAIN * ( forward_p + forward_i + forward_d ); 
            forwardLastError = forwardError; 

            driveMotorL->forward( ( forwardMidMotorSpeed - 1 * forward_g) );
            driveMotorR->forward( ( forwardMidMotorSpeed + 1 * forward_g) );

            Serial.println(forward_g);

        }

        void DrivePID::updateBackwardDrivePID(){
            double backwardError = (double) analogRead(backwardTapeSensorPin1) - analogRead(backwardTapeSensorPin2);

            Serial.println("tp1: " + String( analogRead(this-> backwardTapeSensorPin1)) );
            Serial.println("tp2: " + String( analogRead(this-> backwardTapeSensorPin2)) );
            // Serial.print("error: " );

            backward_p = BACKWARD_P_GAIN * backwardError;
            backward_d = BACKWARD_D_GAIN * (backwardError - backwardLastError);
            backward_i = BACKWARD_I_GAIN * backwardError + backward_i; //const * error + previous int value
            if (backward_i > MAX_BACKWARD_I) {backward_i = MAX_BACKWARD_I;}
            if (backward_i < -MAX_BACKWARD_I) {backward_i = -MAX_BACKWARD_I;}

            backward_g = BACKWARD_LOOP_GAIN * ( backward_p + backward_i + backward_d ); 
 
            driveMotorL->backward( 1 / 1.0 * ( (backwardMidMotorSpeed - 1 * backward_g) ) );
            driveMotorR->backward( 1 / 1.0 * ( (backwardMidMotorSpeed + 1 * backward_g) ) );
        }

        void DrivePID::updateIRDrive(double irError){
            
            /*  Dont need to do this. Just need to tape follow until the ir error is effectively 0  */
            
            double backwardError = irError;

            ir_p = IR_P_GAIN * irError;
            ir_d = IR_D_GAIN * (irError - irLastError);
            ir_i = IR_I_GAIN * irError + ir_i; //const * error + previous int value
            if (ir_i > MAX_IR_I) {ir_i = MAX_IR_I;}
            if (ir_i < -MAX_IR_I) {ir_i = -MAX_IR_I;}

            ir_g = BACKWARD_LOOP_GAIN * ( ir_p + ir_i + ir_d ); 
 
            driveMotorL->backward( ir_g );
            driveMotorR->backward(  1 * ir_g );
        }

}