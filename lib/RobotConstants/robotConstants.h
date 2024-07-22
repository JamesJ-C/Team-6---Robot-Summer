#ifndef ROBOT_CONSTANTS_H 
#define ROBOT_CONSTANTS_H 

#include <Arduino.h>


/*  general robot constants  */
namespace robotConstants {


/*  Motor constants  */
#define MOTOR_FREQUENCY 1000


/*  OLED consts  */
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible



}

/*  Motor pins  */
namespace motorPins {

    /*  Drive motor pins. Adjusted for direction  */
    #define MotorR_P1 PB_6
    #define MotorR_P2 PB_7

    #define MotorL_P1 PB_9
    #define MotorL_P2 PB_8


    /*  ESP PINS  */
    #ifdef ESP32
        #define MOTOR_1_a GPIO32
        #define MOTOR_1_b GPIO33

        #define MTOR_2_a GPIO12
        #define MTOR_2_a GPIO13
    #endif

}

namespace rotaryPins {

    /*  Rotary encoder pins used for testing. May not be the actual control board pins  */
    #define ROTARY_A PB13
    #define ROTARY_B PB14

}


namespace servoPins {



}

namespace analogInPins {

    /*  Pins used for testing  */
    #define POT_PIN A1
    #define IR_SENSOR1 PA_0
    #define IR_SENSOR2 PA_1



}

namespace digitalPins {


    #define BUTTON_PIN PB_12//PA_10

    /*  Adjusted for driving direction  */
    #define FRONT_TAPE_SENSOR_1 PA_7
    #define FRONT_TAPE_SENSOR_2 PA_6

    /*  Not necesarrily adjusted for direction  */
    #define BACK_TAPE_SENSOR_3 PA_0
    #define BACK_TAPE_SENSOR_4 PA_1



}


namespace irConstants {


    #define THRESHOLD 100 // Black line detection threshold
    #define NUM_SAMPLES 200

}



#endif