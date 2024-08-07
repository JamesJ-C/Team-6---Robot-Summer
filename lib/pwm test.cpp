// #include <Arduino.h>
// #include <Wire.h>
// #include <HardwareSerial.h>


/*  libraries we wrote  */
// #include <Motor.h>
// #include <RotaryEncoder.h>
// #include <RobotSystems.h>
// #include <espConstants.h>
//#include <robotConstants.h>
#include <ESp32Servo.h>
// #include <claw.h>

Servo myServo;
Servo myServo2;

void setup() {



    // pinMode(25, OUTPUT);
    pinMode(26, OUTPUT);

myServo.attach(25, 500, 2400);
myServo2.attach(26, 500, 2400);
    
}


void loop() {


// analogWrite(25, 140);

myServo.write(100);
myServo2.write(100);

// analogWrite(26, 140);

} //loop
