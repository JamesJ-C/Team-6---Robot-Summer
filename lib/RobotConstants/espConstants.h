#ifndef ESP_ROBOT_CONSTANTS_H 
#define ESP_ROBOT_CONSTANTS_H 

#include <Arduino.h>


namespace robotControl {

    //threshold value for errors for different motions. used in a while loop for calling the updatepid function 
    #define ERROR_THRESHOLD 30

    //Encoder values for different rotational positions, arbitary values needs tuning 
    #define NINETY_LAZYSUSAN 0
    #define ONE_EIGHTY_LAZYSUSAN 172
    #define TWO_SEVENTY_LAZYSUSAN 340
    
    //Encoder values for different positions of the linear arm movement 
    #define CLAW_FORWARD 100 
    #define CLAW_NEUTRAL 0 
    
    //Servo Positions
    #define FORKLIFTSERVO_READY_POS 90
    #define CLAWSERVO_OPEN_POS 100
    #define CLAWSERVO_CLOSED_POS 50
}

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
    #define LINEAR_ARM_P2 7//25 //12//7 
    //const uint8_t linearArmP1 = 25;
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

#ifdef ESP32
    #define IR_SENSOR_1 34
    #define IR_SENSOR_2 38
#endif
}

#endif