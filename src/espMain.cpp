
#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>


/*  libraries we wrote  */
#include <Motor.h>
#include <RotaryEncoder.h>
#include <RobotSystems.h>
#include <espConstants.h>
//#include <robotConstants.h>
#include <claw.h>

//#ifdef ESP32
#include <ESp32Servo.h>
//#endif


void isrUpdateLinearArmEncoder();
void isrUpdateLazySusanEncoder();


encoder::RotaryEncoder lazySusanEncoder(LAZY_SUSAN_ROTARY_ENCODER_PA, LAZY_SUSAN_ROTARY_ENCODER_PA);
movement::EncodedMotor lazySusanMotor(LAZY_SUSAN_P1, LAZY_SUSAN_P2, &lazySusanEncoder);





void setup() {

    Serial.begin(115200);


}




void loop() {

}


void moveServo(Servo& servo){



} 


void isrUpdateLinearArmEncoder(){

}

void isrUpdateLazySusanEncoder(){
    
}