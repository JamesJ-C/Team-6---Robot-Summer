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

    #define Motor1_P1 PB_0
    #define Motor1_P2 PB_1

    

}

namespace rotaryPins {

    #define ROTARY_A PB8
    #define ROTARY_B PB9

}


namespace servoPins {



}

namespace analogInPins {

    #define POT_PIN A1



}

namespace digitalPins {

    #define BUTTON_PIN PB_12//PA_10

    #define FRONT_TAPE_SENSOR_1 PA_2
    #define FRONT_TAPE_SENSOR_2 PA_3

    #define BACK_TAPE_SENSOR_3 PA_0
    #define BACK_TAPE_SENSOR_4 PA_1



}



#endif