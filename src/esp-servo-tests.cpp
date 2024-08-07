#ifdef ESP32


#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>


/*  libraries we wrote  */
#include <Motor.h>
#include <RotaryEncoder.h>
#include <RobotSystems.h>
#include <espConstants.h>
//#include <robotConstants.h>
#include <ESp32Servo.h>
#include <claw.h>

//#ifdef ESP32

//#endif

void isrUpdateLinearArmEncoder();
void isrUpdateLazySusanEncoder();


encoder::RotaryEncoder lazySusanEncoder(LAZY_SUSAN_ROTARY_ENCODER_PA, LAZY_SUSAN_ROTARY_ENCODER_PB);
movement::EncodedMotor lazySusanMotor(LAZY_SUSAN_P1, LAZY_SUSAN_P2, &lazySusanEncoder);
robot::RobotSubSystem lazySusanSystem(LAZY_SUSAN_LIMIT_SWITCH, -1, &lazySusanMotor);

encoder::RotaryEncoder linearArmEncoder(LINEAR_ARM_ROTARY_ENCODER_PA, LINEAR_ARM_ROTARY_ENCODER_PB);
movement::EncodedMotor linearArmMotor(LINEAR_ARM_P1, LINEAR_ARM_P2, &linearArmEncoder);//
robot::RobotSubSystem linearArmSystem(LINEAR_ARM_LIMIT_SWITCH_A, LINEAR_ARM_LIMIT_SWITCH_B, &linearArmMotor);

robot::IRSensor beaconSensor(IR_SENSOR_1, IR_SENSOR_2);


Servo clawServo;
Servo forkliftServo;
clawActuation::Claw clawSystem(&clawServo, &forkliftServo, CLAW_LIMIT_SWITCH_A, CLAW_LIMIT_SWITCH_B);

enum State{
    START, 
    TRANSITION_TO_4,
    PROCESS_STATION_4,
    PROCESS_STATION_5, 
    TRANSITION_TO_6,
    PROCESS_STATION_6,
    TRANSITION_TO_5,
    TRANSITION_TO_62,
    PROCESS_STATION_62,
    IDLE,
    FINISHED
};
State currentState = IDLE; 

HardwareSerial SerialPort(1);

void setup() {

    delay(2000);

    /*  Serial setup  */
    Serial.begin(115200);
    Serial.println("Setup");
    SerialPort.begin(115200, SERIAL_8N1, RX, TX);


    /*  Servos  */
    clawServo.attach(CLAW_SERVO_PIN);
    forkliftServo.attach(FORKLIFT_SERVO_PIN);

    pinMode(CLAW_SERVO_PIN,OUTPUT);
    pinMode(FORKLIFT_SERVO_PIN,OUTPUT);

    /*  Encoders  */
    pinMode(lazySusanEncoder.getPinA(), INPUT_PULLUP);
    pinMode(lazySusanEncoder.getPinB(), INPUT_PULLUP);

    // pinMode(lazySusanEncoder.getPinA(), INPUT);
    // pinMode(lazySusanEncoder.getPinB(), INPUT);

    pinMode(linearArmEncoder.getPinA(), INPUT_PULLUP);
    pinMode(linearArmEncoder.getPinB(), INPUT_PULLUP);

    /*  Motors  */
    pinMode(lazySusanMotor.getPinA(), OUTPUT);
    pinMode(lazySusanMotor.getPinB(), OUTPUT);

    pinMode(linearArmMotor.getPinA(), OUTPUT);
    pinMode(linearArmMotor.getPinB(), OUTPUT);


    /*  Interrupts  */
    attachInterrupt(lazySusanEncoder.getPinA(), isrUpdateLazySusanEncoder, CHANGE);
    attachInterrupt(lazySusanEncoder.getPinB(), isrUpdateLazySusanEncoder, CHANGE);

    attachInterrupt(digitalPinToInterrupt(linearArmEncoder.getPinA()), isrUpdateLinearArmEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(linearArmEncoder.getPinB()), isrUpdateLinearArmEncoder, CHANGE);
    
}

int val = 0;

int loopCount = 0;

void loop() {

if (loopCount == 0){
    Serial.println("doing the deed");
    // analogWrite(CLAW_SERVO_PIN, 180);
    // analogWrite(FORKLIFT_SERVO_PIN, 180);

//     //clawServo.write(90);
    
    loopCount++;
}
clawServo.write(50);
delay(1000);
for (int i = 50; i < 110; i=i + 1){
    clawServo.write(i);
    delay(10);
}
clawServo.write(100);
delay(1000);
for (int i = 100; i > 50; i=i - 1){
    clawServo.write(i);
    delay(10);
}


} 


void IRAM_ATTR isrUpdateLinearArmEncoder(){

    bool A = digitalRead(linearArmEncoder.getPinA());
    bool B = digitalRead(linearArmEncoder.getPinB());
    linearArmEncoder.updateEncoder(A, B);
}

void IRAM_ATTR isrUpdateLazySusanEncoder(){

    bool A = digitalRead(lazySusanEncoder.getPinA());
    bool B = digitalRead(lazySusanEncoder.getPinB());
    lazySusanEncoder.updateEncoder(A, B);

    //val++;
}

// void isrupdateEncoder() {
//     bool A = digitalRead(LAZY_SUSAN_ROTARY_ENCODER_PA);
//     bool B = digitalRead(LAZY_SUSAN_ROTARY_ENCODER_PB);
//     /*	encodes 2 bit current state  */
//     int encoded = ( A << 1 ) | B;
//     // Serial.println("encoded: " + encoded);
//     // Serial.println(encoded, BIN);
//     /*	encodes the last states bits, concat the current states bits  */
//     int concat = ( lastEncoded << 2 ) | encoded;
//     // Serial.print("concat: ");
//     // Serial.println(concat, BIN);
//     //Serial.println("concat: " + String(concat));
//     /*	hard codes all the possibilities of encoded data  */
//     if (concat == 0b1101 || concat == 0b0100 || concat == 0b0010 || concat == 0b1011){
//        increments++;
//     }
//     if (concat == 0b1110 || concat == 0b0111 || concat == 0b0001 || concat == 0b1000) {
//         increments--;
//     }
//     /*	the current states bits become the next states previous bits  */
//     lastEncoded = encoded;
// }


#endif