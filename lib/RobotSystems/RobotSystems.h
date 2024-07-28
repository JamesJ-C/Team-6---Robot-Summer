#ifndef ROBOT_SYSTEMS_H
#define ROBOT_SYSTEMS_H


#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Motor.h>
#include <robotConstants.h>

namespace robot {


    class RobotSubSystem {

        private:
        uint8_t limit1;
        uint8_t limit2;

        const int loopGain = 1.0;
        const int pGain = 0.55;
        const int iGain = 0.0;
        const int dGain = 1.36;
        double transfer;
        double lastError = 0;
        double iTerm = 0;
        double maxI = 140;

        public:
        movement::EncodedMotor *motor;

        /**
         * @brief Construct a new Robot Sub System object
         * 
         * @param motor motor driving the movement of the system
         * @param limit1 first limit switch attatched to movement
         * @param limit2 second limit switched attatched to movement
         */
        RobotSubSystem (uint8_t limit1, uint8_t limit2, movement::EncodedMotor *motor);

        /**
         * @brief Returns the first limit switch pin
         */
        uint8_t getLimit1();

        /**
         * @brief Returns the second limit switch pin
         */
        uint8_t getLimit2();


        /**
         * @brief performs full movement sweep and initializes rotary encoder values 
         */
        void localize();


        /**
         * @brief rotates motor until encoder reaches the specified value
         * 
         * @param rotaryVal specific rotary encoder value to move to
         */
        void moveToValue(int rotaryVal);


        /**
         * @brief update PID transfer function and send value to motors
         * 
         * @param current current rotary encoder value
         * @param target target encoder value to reach
         */
        void updatePID(int current, int target);

    };





    class DrivePID {


        private:

        uint8_t forwardTapeSensorPin1;
        uint8_t forwardTapeSensorPin2;

        uint8_t backwardTapeSensorPin1;
        uint8_t backwardTapeSensorPin2;

        movement::Motor &driveMotorL;
        movement::Motor &driveMotorR;

        double forward_p;
        double forward_i;
        double forward_d;
        double forward_g;
        double forwardLastError;

        const int forwardMidMotorSpeed = 3300;
        const double FORWARD_LOOP_GAIN = 1.0;
        const double FORWARD_P_GAIN = 0.55;
        const double FORWARD_I_GAIN = 0.0;
        const double FORWARD_D_GAIN = 1.36;
        const double MAX_FORWARD_I = 1400.0;

        double backward_p;
        double backward_i;
        double backward_d;
        double backward_g;
        double backwardLastError;

        const int backwardMidMotorSpeed = 3300;
        const double BACKWARD_LOOP_GAIN = 1.0;
        const double BACKWARD_P_GAIN = 0.55;
        const double BACKWARD_I_GAIN = 0.0;
        const double BACKWARD_D_GAIN = 1.36;
        const double MAX_BACKWARD_I = 1400.0;



        public:

        DrivePID(uint8_t forwardTape1, uint8_t forwardTape2, uint8_t backwardTape1, uint8_t backwardTape2,
        movement::Motor &motorL, movement::Motor motorR); 
        // : forwardTapeSensorPin1(forwardTape1), forwardTapeSensorPin2(forwardTape2), backwardTapeSensorPin1(backwardTape1), 
        // backwardTapeSensorPin2(backwardTape2), driveMotorL(motorL), driveMotorR(motorR) {}

        void updateForwardDrivePID();// {
        //     double forwardError = (double) analogRead(this->forwardTapeSensorPin1) - analogRead(this->forwardTapeSensorPin1);

        //     forward_p = FORWARD_P_GAIN * forwardError;
        //     forward_d = FORWARD_D_GAIN * (forwardError - forwardLastError);
        //     forward_i = FORWARD_I_GAIN * forwardError + forward_i; //const * error + previous int value

        //     if (forward_i > MAX_FORWARD_I) {forward_i = MAX_FORWARD_I;}
        //     if (forward_i < -MAX_FORWARD_I) {forward_i = -MAX_FORWARD_I;}

        //     forward_g = FORWARD_LOOP_GAIN * ( forward_p + forward_i + forward_d ); 
        //     forwardLastError = forwardError; 

        //     driveMotorL.forward( (forwardMidMotorSpeed - 1 * forward_g) );
        //     driveMotorR.forward(  1 / 1.3 * ( ( forwardMidMotorSpeed + 1 * forward_g) ) );

        // }

        void updateBackwardPID(); //{
        //     double backwardError = (double) analogRead(backwardTapeSensorPin1) - analogRead(backwardTapeSensorPin2);

        //     backward_p = BACKWARD_P_GAIN * backwardError;
        //     backward_d = BACKWARD_D_GAIN * (backwardError - backwardLastError);
        //     backward_i = BACKWARD_I_GAIN * backwardError + backward_i; //const * error + previous int value
        //     if (backward_i > MAX_BACKWARD_I) {backward_i = MAX_BACKWARD_I;}
        //     if (backward_i < -MAX_BACKWARD_I) {backward_i = -MAX_BACKWARD_I;}

        //     backward_g = BACKWARD_LOOP_GAIN * ( backward_p + backward_i + backward_d ); 
 
        //     driveMotorL.backward( (backwardMidMotorSpeed - 1 * backward_g) );
        //     driveMotorR.backward(  1 / 1.2 * ( ( backwardMidMotorSpeed + 1 * backward_g) ) );
        // }



    };



}





    #endif

