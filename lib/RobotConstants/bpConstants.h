#ifndef BP_ROBOT_CONSTANTS_H 
#define BP_ROBOT_CONSTANTS_H 

#include <Arduino.h>


namespace constants {

    #define MOTOR_FREQUENCY 100
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
#define TAPE_SENSOR_FORWARD_1 PA6
#define TAPE_SENSOR_FORWARD_2 PA7

#define TAPE_SENSOR_BACKWARD_1 PA2
#define TAPE_SENSOR_BACKWARD_2 PA3

#define TAPE_SENSOR_RIGHT_1 PA4
#define TAPE_SENSOR_RIGHT_2 PA5//not this one

#define TAPE_SENSOR_LEFT_1 PA0
#define TAPE_SENSOR_LEFT_2 PA1


}

namespace limitSwitchPins {

    //limit witch A is bottom; B is top
    #define ELEVATOR_LIMIT_BOTTOM PB12
    #define ELEVATOR_LIMIT_TOP PB15

}

namespace uartPins {
#ifndef ESP32
    #define RX PB11
    #define TX PB10
#endif
}



#endif