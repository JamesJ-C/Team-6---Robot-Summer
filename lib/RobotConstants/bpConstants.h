#ifndef BP_ROBOT_CONSTANTS_H 
#define BP_ROBOT_CONSTANTS_H 

#include <Arduino.h>


namespace constants {

    #define MOTOR_FREQUENCY 500
}


namespace driveMotorPins {

    #define MOTOR_R_P1 PB_6
    #define MOTOR_R_P2 PB_7

    #define MOTOR_L_P1 PB_8
    #define MOTOR_L_P2 PB_9

}

namespace elevatorMotorPins {

    #define ELEVATOR_P1 PA_8
    #define ELEVATOR_P2 PA_9


}

namespace rotaryEncoderPins {

    #define ELEVATOR_ENCODER_PA PB13
    #define ELEVATOR_ENCODER_PB PB14

}

namespace tapeSensorPins {

/*  A forward; B right; C backward; D left  */
#define TAPE_SENSOR_FORWARD_P1 PA6
#define TAPE_SENSOR_FORWARD_P2 PA7

#define TAPE_SENSOR_BACKWARD_P1 PA2
#define TAPE_SENSOR_BACKWARD_P2 PA3

#define TAPE_SENSOR_RIGHT_P1 PA4
#define TAPE_SENSOR_RIGHT_P1 PA5

#define TAPE_SENSOR_LEFT_P1 PA0
#define TAPE_SENSOR_LEFT_P1 PA1


}

namespace limitSwitchPins {

    //limit witch A is bottom; B is top
    #define ELEVATOR_LIMIT_BOTTOM PB12
    #define ELEVATOR_LIMIT_TOP PB15

}

namespace uartPins {

    #define RX PB11
    #define TX PB10

}



#endif