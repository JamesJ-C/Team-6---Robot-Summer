#ifndef ROBOT_CONSTANTS_H 
#define ROBOT_CONSTANTS_H 

#include <Arduino.h>


/*  general robot constants  */
namespace robotConstants {


/*  Motor constants  */
#define MOTOR_FREQUENCY 500


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

    /*  Elevator motor  */
    #define MOTOR_ELEVATOR_P1 PB_6
    #define MOTOR_ELEVATOR_P2 PB_7

    /*  Linear arm motion pins  */
    #define ARM_MOTOR_P1 PB_8
    #define ARM_MOTOR_P2 PB_9

    #define LAZY_SUSAN_MOTOR_P1 PB_9
    #define LAZY_SUSAN_MOTOR_P2 PB_8


    /*  ESP PINS  */
    #ifdef ESP32
        #define MOTOR_1_a 32
        #define MOTOR_1_b 33

        #define MTOR_2_a 12
        #define MTOR_2_a 13
    #endif

}

namespace rotaryPins {

    /*  Rotary encoder pins used for testing. May not be the actual control board pins  */
    // #define ROTARY_A PB13
    // #define ROTARY_B PB14

    #define ARM_ROTARY_ENCODER_A PB13
    #define ARM_ROTARY_ENCODER_B PB14
    
    #define ELEVATOR_ROTARY_ENCODER_A PB13
    #define ELEVATOR_ROTARY_ENCODER_B PB14

    #define LAZY_SUSAN_ROTARY_ENCODER_A PB13
    #define LAZY_SUSAN_ROTARY_ENCODER_B PB14

}


namespace servoPins {



}

namespace analogInPins {

    /*  Pins used for testing  */
    //#define POT_PIN A1
    #define IR_SENSOR1 PA_0
    #define IR_SENSOR2 PA_1
    
    /* Pins used for station detection */
    #define TAPE_LA PA4
    #define TAPE_LB PA5


}

namespace digitalPins {


    // #define BUTTON_PIN PB_12//PA_10

    /*  Adjusted for driving direction  */
    #define FRONT_TAPE_SENSOR_1 PA_7
    #define FRONT_TAPE_SENSOR_2 PA_6

    /*  Adjusted for direction  */
    #define BACK_TAPE_SENSOR_1 PA_0
    #define BACK_TAPE_SENSOR_2 PA_1

    /* Station tape detectors */
    #define RIGHT_TAPE_SENSOR_1 PA_5
    #define RIGHT_TAPE_SENSOR_2 PA_4
    #define LEFT_TAPE_SENSOR_1 PA_3
    #define LEFT_TAPE_SENSOR_2 PA_2


    /* Limit switches for elevator */
    #define ELEVATOR_LOWER_LIMIT_SWITCH PB_12
    #define ELEVATOR_UPPER_LIMIT_SWITCH PB_15

    /* Limit switches for elevator */
    #define ARM_LOWER_LIMIT_SWITCH PB12
    #define ARM_UPPER_LIMIT_SWITCH PB15

    #define LAZY_SUSAN_LIMIT_SWITCH PB15


}


namespace irConstants {


    #define THRESHOLD 100 // Black line detection threshold
    #define NUM_SAMPLES 200

}



#endif