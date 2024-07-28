
#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>


/*  libraries we wrote  */
#include <Motor.h>
#include <RotaryEncoder.h>
#include <RobotSystems.h>
#include <robotConstants.h>



/*  Object declerations  */
//movement::Motor motorL(MOTOR_L_P1, MOTOR_L_P2);
//movement::Motor motorR(MOTOR_R_P1, MOTOR_R_P2);

//movement::EncodedMotor ElevatorMotor(ELEVATOR_P1, ELEVATOR_P2);

robot::RobotSubSystem Elevator();

void setup() {

    Serial.begin(115200);


}


void loop() {

}


