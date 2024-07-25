#include <Arduino.h>
#include <Servo.h>
#include <HardwareSerial.h>
#include "RotaryEncoder.h"
#include "Motor.h"
#include "robotConstants.h"


#define TAPE_THRESHOLD 800 
#define MARKER_SENSOR_L PB
#define MARKER_SENSOR_R PB


HardwareSerial SerialPort(USART3);
String msg;

movement::Motor MotorL(MotorL_P1, MotorL_P2);
movement::Motor MotorR(MotorR_P1, MotorR_P2);

enum State{
    IDLE,
    STATION_1,
    STATION_2,
    STATION_3,
    STATION_4
};
State currentState = IDLE; 

int markerCount = 0; 
const int markerAtStation[] = {1,2,3,4}; 

void setup(){
    pinMode(MotorL.getPinA(), OUTPUT);
    pinMode(MotorL.getPinB(), OUTPUT);
    pinMode(MotorR.getPinA(), OUTPUT);
    pinMode(MotorR.getPinB(), OUTPUT);

    pinMode(MARKER_SENSOR_L,INPUT);
    pinMode(MARKER_SENSOR_R, INPUT); 
    Serial.begin(115200);
}

void loop{
    pidDriving(); 

    if(markerDetected()){
        markercount++;
        delay(100); 
    }

    switch (c
    {
    case /* constant-expression */:
        /* code */
        break;
    
    default:
        break;
    }
}

bool markerDetected(){
    return (analogRead(MARKER_SENSOR_L) >= TAPE_THRESHOLD || analogRead(MARKER_SENSOR_R) >= TAPE_THRESHOLD)
}


