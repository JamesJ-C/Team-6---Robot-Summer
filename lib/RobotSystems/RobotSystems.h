#ifndef ROBOT_SYSTEMS_H
#define ROBOT_SYSTEMS_H


#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Motor.h>

#ifdef ESP32
#include <bpConstants.h>
#endif

#ifndef ESP32
#include <espConstants.h>
#endif

#include <vector>

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

        double ir_p;
        double ir_i;
        double ir_d;
        double ir_g;
        double irLastError;

        const double IR_LOOP_GAIN = 1.0;
        const double IR_P_GAIN = 0.55;
        const double IR_I_GAIN = 0.0;
        const double IR_D_GAIN = 1.36;
        const double MAX_IR_I = 1400.0;



        public:

        DrivePID(uint8_t forwardTape1, uint8_t forwardTape2, uint8_t backwardTape1, uint8_t backwardTape2,
        movement::Motor &motorL, movement::Motor motorR); 

        void updateForwardDrivePID();

        void updateBackwardPID(); 

        void updateIRDrive(double irError);

    };


    class IRSensor {



        double crossCorrelation (uint8_t analogPin){

            std::vector<double> IRsignal;

            int numSamples = 0;
            unsigned long finishTime = 0;
            unsigned long startTime = millis();

            while (millis() - startTime < 10){

                IRsignal.push_back(analogRead(analogPin));
                numSamples++;
                finishTime = millis();
            }


            double oneK[2* numSamples] = {0};
            double oneKCorr[numSamples] = {0};

            int dt = ( finishTime - startTime );
            double oneKT = (double) numSamples / ( (double) dt );

            for(int i = 0; i < 2 * numSamples;  i++) {
            
                oneK[i] = sin(i * TWO_PI / oneKT);
            
            }

            for (int k = 0; k < numSamples; k++){

                oneKCorr[k] = 0;

                for (int i = 0; i < numSamples; i++){      
                oneKCorr[k] += IRsignal.at(i) * oneK[k+i];
                }

            }

            double max = oneKCorr[0];

            for (int i=0; i< numSamples; i++) {

                if (oneKCorr[i]>max){
                max = oneKCorr[i];
                }
            }

            // if (max < minTot){
            //     minTot = max;
            // }
            // if (max > maxTot){
            //     maxTot = max;
            // }
            // // avg = ( (loopCount - 1) * avg + max ) / loopCount;

            return max;

            }



        std:: vector<double> bothCrossCorrelation (uint8_t analogPin1, uint8_t analogPin2){

            std::vector<double> IRsignal1;
            std::vector<double> IRsignal2;

            int numSamples = 0;
            unsigned long finishTime = 0;
            unsigned long startTime = millis();

            while (millis() - startTime < 10){

                IRsignal1.push_back(analogRead(analogPin1));
                IRsignal2.push_back(analogRead(analogPin2));

                numSamples++;
                finishTime = millis();
            }


            double oneK[2 * numSamples] = {0};
            double oneKCorr1[numSamples] = {0};
            double oneKCorr2[numSamples] = {0};


            int dt = ( finishTime - startTime );
            double oneKT = (double) numSamples / ( (double) dt );

            for(int i = 0; i < 2 * numSamples;  i++) {
            
                oneK[i] = sin(i * TWO_PI / oneKT);
            
            }

            for (int k = 0; k < numSamples; k++){

                oneKCorr1[k] = 0;

                for (int i = 0; i < numSamples; i++){      
                oneKCorr1[k] += IRsignal1.at(i) * oneK[k+i];
                oneKCorr2[k] += IRsignal2.at(i) * oneK[k+i];
                }

            }

            double max1 = oneKCorr1[0];
            double max2 = oneKCorr2[0];

            for (int i=0; i< numSamples; i++) {

                if (oneKCorr1[i]>max1){
                max1 = oneKCorr1[i];
                }
                if (oneKCorr2[i]>max2){
                max2 = oneKCorr2[i];
                }
            }

            std::vector<double> result;

            result.push_back(max1);
            result.push_back(max2);

            return result;

        }


    };




}





    #endif

