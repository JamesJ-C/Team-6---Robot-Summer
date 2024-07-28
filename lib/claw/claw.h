#ifndef ROBOT_CLAW_H
#define ROBOT_CLAW_H

#include <Arduino.h>
#include <ESP32Servo.h>

namespace clawActuation {


    class Claw {

        private:
        uint8_t limit1;
        uint8_t limit2;
        int clawOpen = 0;
        int clawClosed = 100;
        int forkliftDeployed = 100;
        int forkliftFolded = 0;

        public:
        Servo *clawServo;
        Servo *forkliftServo;
        
        /**
         * @brief Construct a new Claw object
         * 
         * @param clawServo servo driving the claw grabbing movement
         * @param forkliftServo servo driving the forklift movement
         * @param limit1 first limit switch on the claw to detect if an object is grabbed
         * @param limit2 first limit switch on the claw to detect if an object is grabbed
         */
        Claw (Servo *clawServo, Servo *forkliftServo, uint8_t limit1, uint8_t limit2);

        /**
         * @brief Returns the first limit switch pin
         */
        uint8_t getLimit1();

        /**
         * @brief Returns the second limit switch pin
         */
        uint8_t getLimit2();

        // Mutator functions for open/closed servo positions
        void setClawOpen(int clawOpen);

        void setClawClosed (int clawClosed);

        void setForkliftDeployed (int forkliftDeployed);

        void setForkliftFolded (int forkliftFolded);

        /**
         * @brief opens the claw
         * 
         */
        void openClaw();


        /**
         * @brief closes the claw to grab an object, retries if object is not detected
         * 
         */
        void closeClaw();

        /**
         * @brief deploys forklift system
         * 
         */
        void deployForklift();


        /**
         * @brief folds forklift system back up
         * 
         */
        void foldForklift();

    };


}

#endif