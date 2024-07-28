#ifndef ESP_ROBOT_CONSTANTS_H 
#define ESP_ROBOT_CONSTANTS_H 

#include <Arduino.h>


namespace constants {

    #define MOTOR_FREQUENCY 500
    /*  OLED consts  */
    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 64 // OLED display height, in pixels
    #define OLED_RESET 	-1 // This display does not have a reset pin accessible

}

namespace lazySusanPins {

    #define LAZY_SUSAN_P1 32
    #define LAZY_SUSAN_P2 33

}

namespace linearArm {
    
    //Need to verify 7 works as a pin
    #define LINEAR_ARM_P1 13
    #define LINEAR_ARM_P2 7 
}

namespace rotaryEncoders {

    //NEED TO VERIFY THESE PINS WORK
    #define LAZY_SUSAN_ROTARY_ENCODER_PA 2
    #define LAZY_SUSAN_ROTARY_ENCODER_PB 4

    //NEED TO VERIFY THESE PINS WORK
    #define LINEAR_ARM_ROTARY_ENCODER_PA 3
    #define LINEAR_ARM_ROTARY_ENCODER_PB 35


}

namespace limitSwitches {

//C 27 x
//D 12
//E 14 x
//F 19 x
//G 8 x
//H 5 x
//I TX0

    #define LAZY_SUSAN_LIMIT_SWITCH 27

    //E is the extended position limit switch; F is the retracted position limit switch
    #define LINEAR_ARM_LIMIT_SWITCH_A 14
    #define LINEAR_ARM_LIMIT_SWITCH_B 19

    /*NEED TO VERIFY THESE PINS WORK*/
    #define CLAW_LIMIT_SWITCH_A 8
    #define CLAW_LIMIT_SWITCH_B 5



}

namespace servoPins {

//25
//26
    #define CLAW_SERVO_PIN 25
    #define FORKLIFT_SERVO_PIN 26

}

namespace uartPins {
    
    #define RX 9
    #define TX 10

}

namespace irPins {

    #define IR_SENSOR_1 34
    #define IR_SENSOR_1 38

}

#endif