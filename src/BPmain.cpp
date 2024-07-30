
#ifndef ESP32

#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>


/*  libraries we wrote  */
#include <Motor.h>
#include <RotaryEncoder.h>
#include <RobotSystems.h>
//#include <robotConstants.h>
#include <bpConstants.h>

#define TAPE_THRESHOLD 800 
#define MARKER_SENSOR_L PB
#define MARKER_SENSOR_R PB


//Encoder values for different heights of the elevator 
#define FORKLIFT_COUNTER_HEIGHT 1000
#define FORKLIFT_SECURE_HEIGHT 1001 //A height that is just a little bit taller than counter height
#define CLAW_COUNTER_HEIGHT 900
#define CLAW_SECURE_HEIGHT 901

void isrUpdateElevatorEncoder();


/*  Object declerations  */

movement::Motor motorL(MOTOR_L_P1, MOTOR_L_P2);
movement::Motor motorR(MOTOR_R_P1, MOTOR_R_P2);

encoder::RotaryEncoder elevatorEncoder(ELEVATOR_ENCODER_PA, ELEVATOR_ENCODER_PB);
movement::EncodedMotor ElevatorMotor(ELEVATOR_P1, ELEVATOR_P2, &elevatorEncoder);

//robot::RobotSubSystem Elevator();
robot::RobotSubSystem ElevatorSystem(ELEVATOR_LIMIT_BOTTOM, ELEVATOR_LIMIT_TOP, &ElevatorMotor);

robot::DrivePID driveSystem(TAPE_SENSOR_FORWARD_2, TAPE_SENSOR_FORWARD_1, TAPE_SENSOR_BACKWARD_1, TAPE_SENSOR_BACKWARD_2, &motorL, &motorR);


HardwareSerial SerialPort(USART3);

enum State{
    START, 
    TRANSITION_TO_4,
    PROCESS_STATION_4, 
    TRANSITION_TO_6,
    PROCESS_STATION_6,
    TRANSITION_TO_5,
    TRANSITION_TO_62,
    PROCESS_STATION_62
};

void setup() {

    delay(2000);

    Serial.begin(115200);
    Serial.println("setup");
    SerialPort.begin(115200);

    pinMode(elevatorEncoder.getPinA(), INPUT_PULLUP);
    pinMode(elevatorEncoder.getPinB(), INPUT_PULLUP);

    pinMode(motorL.getPinA(), OUTPUT);
    pinMode(motorL.getPinB(), OUTPUT);

    pinMode(driveSystem.forwardTapeSensorPin1, INPUT);
    pinMode(driveSystem.forwardTapeSensorPin2, INPUT);
    pinMode(driveSystem.backwardTapeSensorPin1, INPUT);
    pinMode(driveSystem.backwardTapeSensorPin2, INPUT);


    attachInterrupt(digitalPinToInterrupt(elevatorEncoder.getPinA()), isrUpdateElevatorEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(elevatorEncoder.getPinB()), isrUpdateElevatorEncoder, CHANGE);

    // ElevatorSystem.localize();

    motorL.forward(3000);
    delay(200);
    motorR.forward(3000);
    delay(200);
    motorL.off();
    motorR.off();
    delay(1000);

}


void loop() {

    driveSystem.updateForwardDrivePID();

}

void isrUpdateElevatorEncoder(){

    bool A = digitalRead(elevatorEncoder.getPinA());
    bool B = digitalRead(elevatorEncoder.getPinA());
    elevatorEncoder.updateEncoder(A, B);

}

#endif