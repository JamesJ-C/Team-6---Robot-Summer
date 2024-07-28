#include <Arduino.h>
#include <ESP32Servo.h>
#include <claw.h>

namespace clawActuation {

        /**
         * @brief Construct a new Claw object
         * 
         * @param clawServo servo driving the claw grabbing movement
         * @param forkliftServo servo driving the forklift movement
         * @param limit1 first limit switch on the claw to detect if an object is grabbed
         * @param limit2 first limit switch on the claw to detect if an object is grabbed
         */
        Claw::Claw (Servo *clawServo, Servo *forkliftServo, uint8_t limit1, uint8_t limit2) {
            this->clawServo = clawServo;
            this->forkliftServo = forkliftServo;
            this->limit1 = limit1;
            this->limit2 = limit2;
        }

        /**
         * @brief Returns the first limit switch pin
         */
        uint8_t Claw::getLimit1() {
            return limit1;
        }

        /**
         * @brief Returns the second limit switch pin
         */
        uint8_t Claw::getLimit2() {
            return limit2;
        }

        // Mutator functions for open/closed servo positions
        void Claw::setClawOpen(int open) {
            clawOpen = open;
        }

        void Claw::setClawClosed (int closed) {
            clawClosed = closed;
        }

        void Claw::setForkliftDeployed (int deployed) {
            forkliftDeployed = deployed;
        }

        void Claw::setForkliftFolded (int folded) {
            forkliftFolded = folded;
        }

        /**
         * @brief opens the claw
         * 
         */
        void Claw::openClaw() {
            this->clawServo->write(clawOpen);
        }


        /**
         * @brief closes the claw to grab an object, retries if object is not detected
         * 
         */
        void Claw::closeClaw() {
            this->clawServo->write(clawClosed);
        }

        /**
         * @brief deploys forklift system
         * 
         */
        void Claw::deployForklift() {
            this->forkliftServo->write(forkliftDeployed);
        }


        /**
         * @brief folds forklift system back up
         * 
         */
        void Claw::foldForklift() {
            this->forkliftServo->write(forkliftFolded);
        }

};

