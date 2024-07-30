
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


void isrUpdateElevatorEncoder();


/*  Object declerations  */

movement::Motor motorL(MOTOR_L_P1, MOTOR_L_P2);
movement::Motor motorR(MOTOR_R_P1, MOTOR_R_P2);

encoder::RotaryEncoder elevatorEncoder(ELEVATOR_ENCODER_PA, ELEVATOR_ENCODER_PB);
// movement::EncodedMotor ElevatorMotor(ELEVATOR_P1, ELEVATOR_P2, &elevatorEncoder);
movement::EncodedMotor ElevatorMotor(MOTOR_L_P1, MOTOR_L_P2, &elevatorEncoder);

//robot::RobotSubSystem Elevator();
robot::RobotSubSystem ElevatorSystem(ELEVATOR_LIMIT_BOTTOM, ELEVATOR_LIMIT_TOP, &ElevatorMotor);


HardwareSerial SerialPort(USART3);

void setup() {

    delay(2000);

    Serial.begin(115200);
    Serial.println("setup");
    SerialPort.begin(115200);

    pinMode(elevatorEncoder.getPinA(), INPUT_PULLUP);
    pinMode(elevatorEncoder.getPinB(), INPUT_PULLUP);

    Serial.println("pinA: " + String (elevatorEncoder.getPinA()));
    Serial.println("pinB: " + String (elevatorEncoder.getPinB()));


    attachInterrupt(digitalPinToInterrupt(elevatorEncoder.getPinA()), isrUpdateElevatorEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(elevatorEncoder.getPinB()), isrUpdateElevatorEncoder, CHANGE);

    // ElevatorSystem.localize();


    /*  Wait for confirmation from the BP that setup is good  */
    // while (true){ 
    //     if ( SerialPort.parseInt() == 1){
    //         SerialPort.println(1);
    //     }
    // }

}

int callCount = 0;
int lastEncoded = 0;

void loop() {

    if(elevatorEncoder.getIncrements() != lastEncoded){
    Serial.println("enc: " + String(    elevatorEncoder.getIncrements() ) );
    lastEncoded = elevatorEncoder.getIncrements();
    }
    
    //Serial.println("Call count: " + String(    callCount ) );
    // ElevatorSystem.updatePID(80);

    // ElevatorMotor.forward(2000);
    // delay(1000);
    // ElevatorMotor.off();
    // delay(500);
    // ElevatorMotor.backward(2000);
    // delay(1000);

}


void isrUpdateElevatorEncoder(){
    callCount++;
    // bool A = digitalRead(elevatorEncoder.getPinA());
    // bool B = digitalRead(elevatorEncoder.getPinA());
    elevatorEncoder.updateEncoder(0, 0);

}


#endif