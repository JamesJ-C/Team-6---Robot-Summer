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

        bool singleLimitSwitch = false;

        double loopGain = 1.0;//10.0;//0.8; for the arm
        double pGain = 6.2;
        double iGain = 0.5;//0.68;
        double dGain = 2.1;//2.0;//1.8;
        double transfer;
        double lastError = 0;
        double motor_i = 0;
        double maxI = 2720;//2800;//3000;

        public:
        movement::EncodedMotor *motor;

        volatile bool firstSwitchHit = false;
        volatile bool secondSwitchHit = false;

        /**
         * @brief Construct a new Robot Sub System object
         * 
         * @param motor motor driving the movement of the system
         * @param limit1 first limit switch attatched to movement
         * @param limit2 second limit switched attatched to movement
         */
        RobotSubSystem (uint8_t limit1, uint8_t limit2, movement::EncodedMotor *motor);

        RobotSubSystem (uint8_t limit1, uint8_t limit2, movement::EncodedMotor *motor, 
        double pGain, double iGain, double dGain, double loopGain);

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
        void localize(const int motorSpeedForward, const int motorSpeedBackward);


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
        int updatePID(int target);

    };





    class DrivePID {


        public:

        uint8_t forwardTapeSensorPin1;
        uint8_t forwardTapeSensorPin2;

        uint8_t backwardTapeSensorPin1;
        uint8_t backwardTapeSensorPin2;

private:
        movement::Motor *driveMotorL;
        movement::Motor *driveMotorR;

        double forward_p;
        double forward_i;
        double forward_d;
        double forward_g;
        double forwardLastError;

        const int forwardMidMotorSpeed = 3850;//33
        const double FORWARD_LOOP_GAIN = 1.0;
        const double FORWARD_P_GAIN = 0.9;
        const double FORWARD_I_GAIN = 0.0;
        const double FORWARD_D_GAIN = 1.9;
        const double MAX_FORWARD_I = 1400.0;

        double backward_p;
        double backward_i;
        double backward_d;
        double backward_g;
        double backwardLastError;

        const int backwardMidMotorSpeed = 3800;//33
        const double BACKWARD_LOOP_GAIN = 1.0;
        const double BACKWARD_P_GAIN = 0.8;
        const double BACKWARD_I_GAIN = 0.0;
        const double BACKWARD_D_GAIN = 1.0;
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
        movement::Motor *motorL, movement::Motor *motorR); 

        void updateForwardDrivePID();

        void updateBackwardDrivePID(); 

        void updateIRDrive(double irError);

    };


    class IRSensor {

        protected:

        uint8_t analogPin1;
        uint8_t analogPin2;

        std::vector<int> IRsignalPin1;
        std::vector<int> IRsignalPin2;
        int numSamples = 0;
        unsigned long startTime;
        unsigned long finishTime;

        double maxPin1;
        double maxPin2;

        public:
        const int MAX_NUM_SAMPLES = 600;

        public:
        /**
         * @brief Construct a new IRSensor object
         * 
         * @param pin1 analogPin 1
         * @param pin2 analogPin 2
         */
        IRSensor(uint8_t pin1, uint8_t pin2) : analogPin1(pin1), analogPin2(pin2) {}

        uint8_t getPin1() const {
            return analogPin1;
        }

        uint8_t getPin2() const {
            return analogPin2;
        }

        uint8_t getmax1() const {
            return maxPin1;
        }

        uint8_t getmax2() const {
            return maxPin2;
        }

        /**
         * @brief Calculates the cross correlation of 1 analog pin
         * 
         * @param analogPin analog pin to read from
         * @return double cross correlation value 
         */
        double crossCorrelation (uint8_t analogPin){

            std::vector<double> IRsignal;

            int numSamples = 0;
            unsigned long finishTime = 0;
            unsigned long startTime = millis();

            while (millis() - startTime < 100){

                IRsignal.push_back(analogRead(analogPin));
                numSamples++;
                finishTime = millis();
                Serial.println(IRsignal.at(numSamples-1));
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


            Serial.println("max: " + String (max));
            return max;

            }


        /**
         * @brief calculates the cross correlation values for 2 pins 
         * 
         * @param analogPin1 pin 1 connected to IR sensor
         * @param analogPin2 pin 2 connected to the IR sensor
         * @return std::vector<double> 2 item vector with pin1's reading first followed by pin2's reading
         */
        std::vector<double>* bothCrossCorrelation (uint8_t analogPin1, uint8_t analogPin2){

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

            for (int i=0; i < numSamples; i++) {

                if (oneKCorr1[i]>max1){
                max1 = oneKCorr1[i];
                }
                if (oneKCorr2[i]>max2){
                max2 = oneKCorr2[i];
                }
            }

    Serial.println("result1 act: " + String ( max1 ));
    Serial.println("result2 act: " + String ( max2) );
    Serial.println();

            std::vector<double> *result = new std::vector<double>;// = {max1, max2};

            //std::vector<double> *result = &vec;
            result->push_back(max1);
            result->push_back(max2);

            return result;

        }

        /**
         * @brief samples or calcualtes the cross correlation of 2 analog pins
         * 
         * @return int 0 if the data has not been updated, 1 if the data has been updated
         */
        int updateCorrelation(){
            if (numSamples = 0){
                startTime = millis();
                IRsignalPin1.push_back(analogRead(analogPin1));
                IRsignalPin2.push_back(analogRead(analogPin2));
                numSamples++;
            }
            else if (numSamples < MAX_NUM_SAMPLES){
                IRsignalPin1.push_back(analogRead(analogPin1));
                IRsignalPin2.push_back(analogRead(analogPin2));
                numSamples++;
            }
            if (numSamples >= MAX_NUM_SAMPLES){
                finishTime = millis();
                calcCrossCorrelation();
                resetNums();
                return 1;
            }
        return 0;

        }

       protected: 
        /**
         * @brief calculates the crosscorrelation for the IRsignalPinX vectors
         * 
         */
        void calcCrossCorrelation() {

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
                oneKCorr1[k] += IRsignalPin1.at(i) * oneK[k+i];
                oneKCorr2[k] += IRsignalPin2.at(i) * oneK[k+i];
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

            maxPin1 = max1;
            maxPin2 = max2;
                      
        }

        /**
         * @brief resets all the nums used in the cross correlation code
         * 
         */
        void resetNums(){

            IRsignalPin1.clear();
            IRsignalPin2.clear();
            numSamples = 0;
            startTime = 0;
            finishTime = 0;

        }
    };




}





    #endif

