#ifndef ROBOT_SUB_SYSTEM
#define ROBOT_SUB_SYSTEM


#include <Arduino.h>
#include <RotaryEncoder.h>
#include <Motor.h>

namespace robot {

    #ifdef ESP32
        #include <espConstants.h>
    #endif

    #ifndef ESP32
        #include <bpConstants.h>
    #endif

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
        movement::Motor *motor;

        /**
         * @brief Construct a new Robot Sub System object
         * 
         * @param motor motor driving the movement of the system
         * @param limit1 first limit switch attatched to movement
         * @param limit2 second limit switched attatched to movement
         */
        RobotSubSystem (uint8_t limit1, uint8_t limit2, movement::Motor *motor);

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
}
    #endif

