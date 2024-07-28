
#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>


/*  libraries we wrote  */
#include <Motor.h>
#include <RotaryEncoder.h>
#include <RobotSystems.h>
#include <robotConstants.h>
#include <claw.h>

//#ifdef ESP32
#include <ESp32Servo.h>
//#endif


void moveServo(Servo& servo);


void setup() {

    Serial.begin(115200);


}




void loop() {

}


void moveServo(Servo& servo){



} 
