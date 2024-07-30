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
#include <claw.h>

//#ifdef ESP32
#include <ESp32Servo.h>
//#endif

//Encoder values for different rotational positions, arbitary values needs tuning 
#define NINETY_LAZYSUSAN 666
#define ONE_EIGHTY_LAZYSUSAN 667
#define TWO_SEVENTY_LAZYSUSAN 668

//Encoder values for different positions of the linear arm movement 
#define CLAW_FORWARD 100 
#define CLAW_NEUTRAL 50 

//Servo Positions
#define FORKLIFTSERVO_READY_POS 90
#define CLAWSERVO_OPEN_POS 120
#define CLAWSERVO_CLOSED_POS 0

//Receive and transmit pins 
#define RX_PIN PA 
#define TX_PIN PA 

void isrUpdateLinearArmEncoder();
void isrUpdateLazySusanEncoder();

encoder::RotaryEncoder lazySusanEncoder(LAZY_SUSAN_ROTARY_ENCODER_PA, LAZY_SUSAN_ROTARY_ENCODER_PB);
movement::EncodedMotor lazySusanMotor(LAZY_SUSAN_P1, LAZY_SUSAN_P2, &lazySusanEncoder);
robot::RobotSubSystem lazySusanSystem(LAZY_SUSAN_LIMIT_SWITCH, -1, &lazySusanMotor);

encoder::RotaryEncoder linearArmEncoder(LINEAR_ARM_ROTARY_ENCODER_PA, LINEAR_ARM_ROTARY_ENCODER_PB);
movement::EncodedMotor linearArmMotor(LINEAR_ARM_P1, LINEAR_ARM_P2, &linearArmEncoder);
robot::RobotSubSystem linearArmSystem(LINEAR_ARM_LIMIT_SWITCH_A, LINEAR_ARM_LIMIT_SWITCH_B, &linearArmMotor);

Servo clawServo;
Servo forkliftServo;
clawActuation::Claw clawSystem(&clawServo, &forkliftServo, CLAW_LIMIT_SWITCH_A, CLAW_LIMIT_SWITCH_B);

enum State{
    START, 
    PROCESS_STATION_4, 
    PROCESS_STATION_6,
    PROCESS_STATION_5
    FINISHED
}
State currentState = IDLE; 

HardwareSerial SerialPort(1);

void forward(){
    analogWrite(LINEAR_ARM_P1, 200);
    analogWrite(LINEAR_ARM_P2, 0);
}

void backward(){
    analogWrite(LINEAR_ARM_P2, 200);
    analogWrite(LINEAR_ARM_P1, 0);
}

void setup() {

    delay(2000);

    /*  Serial setup  */
    Serial.begin(115200);
    Serial.println("Setup");
    SerialPort.begin(115200, SERIAL_8N1, RX, TX);

    /*  Servos  */
    clawServo.attach(CLAW_SERVO_PIN);
    forkliftServo.attach(FORKLIFT_SERVO_PIN);

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



    // lazySusanSystem.localize();
    // delay(100);
    //linearArmSystem.localize();
    delay(100);
    //}
    // while (true){ 
    //     if ( (int) SerialPort.read() = 1){
    //         break;
    //     }
    // }
    delay(1000);
    linearArmMotor.off();
}

int val = 0;
bool A;
bool B;

int lastEncoded = 0x00;
int increments = 0;

void loop() {

switch (currentState)
{
case START:
forkliftServo.write(FORKLIFTSERVO_READY_POS);
if(SerialPort.available()){
    int receivedVal = SerialPort.parseInt();
    if(receivedVal == 1){ // 1 is the signal indicating that the BP has finished driving 
    currentState = PROCESS_STATION_4; 
    }
    break; 
}
case PROCESS_STATION_4: 
 lazySusanSystem.moveToValue(NINETY_LAZYSUSAN); 
 SerialPort.println(2); 
 //wait for bp to adjust height 
 if(SerialPort.available()){
    int receivedVal = SerialPort.parseInt();
    if(receivedVal == 3){
    linearArmSystem.moveToValue(CLAW_FORWARD); 
    Serial.println(4); 
    }
 }
 if(SerialPort.available()){
    int receivedVal = SerialPort.parseInt();
    if(receivedVal == 1)
    {
        currentState = PROCESS_STATION_6;
    }
case PROCESS_STATION_6:
    lazySusanSystem.moveToValue(TWO_SEVENTY_LAZYSUSAN);
    Serial.println(2);
    if(SerialPort.available()){
        int receivedVal = SerialPort.parseInt(); 
        if(receivedVal == 3){
            linearArmSystem.moveToValue(CLAW_NEUTRAL);
            Serial.println(4);
        }
    }
    f(SerialPort.available()){
        int receivedVal = SerialPort.parseInt(); 
        if(receivedVal == 1){
            currentState = PROCESS_STATION_5; 
        }
    break;
case PROCESS_STATION_5:
    lazySusanSystem.moveToValue(TWO_SEVENTY_LAZYSUSAN);
    clawServo.write(CLAW_OPEN_POS);
    SerialPort.println(2); 
    }
    if(SerialPort.available()){
        int receivedVal = SerialPort.parseInt();
        if(receivedVal == 3){
            linearArmSystem.moveToValue(CLAW_FORWARD); 
            clawServo.write(CLAWSERVO_CLOSED_POS);
            SerialPort.write(5); 
        }
    }
    if(SerialPort.avaliable()){
    receivedVal = SerialPort.parseInt(); 
    if(receivedVal == 1){
        currentState = PROCESS_STATION_62;
    }
}
case PROCESS_STATION_62:
    clawServo.write(CLAW_OPEN_POS); 
    linearArmSystem.moveToValue(CLAW_NEUTRAL); 
    currentState = FINISHED; 

case FINISHED: 
// tbd 
}


    // lazySusanSystem.updatePID(80);
    //Serial.println("LS enc: " + String( lazySusanEncoder.getIncrements() ) );

    // analogWrite(12, 200);
    //     analogWrite(25, 200);

    //linearArmSystem.updatePID(80);
    Serial.println("LAenc: " + String( linearArmEncoder.getIncrements() ) );

        linearArmMotor.forward(200);
        delay(1000);
        linearArmMotor.backward(200);
        delay(1000);

        // forward();
        // delay(1000);
        // backward();
        // delay(1000);
    //}


}


//  put a if statement on the whole FSM if transitionisReady && ESP done 
//usee wire and send 1, when ready. 

void IRAM_ATTR isrUpdateLinearArmEncoder(){

    bool A = digitalRead(linearArmEncoder.getPinA());
    bool B = digitalRead(linearArmEncoder.getPinB());
    linearArmEncoder.updateEncoder(A, B);
}

void IRAM_ATTR isrUpdateLazySusanEncoder(){

    A = digitalRead(lazySusanEncoder.getPinA());
    B = digitalRead(lazySusanEncoder.getPinB());
    lazySusanEncoder.updateEncoder(A, B);

    //val++;
}


void isrupdateEncoder(){
        bool A = digitalRead(LAZY_SUSAN_ROTARY_ENCODER_PA);
        bool B = digitalRead(LAZY_SUSAN_ROTARY_ENCODER_PB);

        /*	encodes 2 bit current state  */
        int encoded = ( A << 1 ) | B;

        // Serial.println("encoded: " + encoded);
        // Serial.println(encoded, BIN);
        /*	encodes the last states bits, concat the current states bits  */
        int concat = ( lastEncoded << 2 ) | encoded;
        // Serial.print("concat: ");
        // Serial.println(concat, BIN);

        //Serial.println("concat: " + String(concat));
        /*	hard codes all the possibilities of encoded data  */
        if (concat == 0b1101 || concat == 0b0100 || concat == 0b0010 || concat == 0b1011){
            increments++;
        }
        
        if (concat == 0b1110 || concat == 0b0111 || concat == 0b0001 || concat == 0b1000) {
            increments--;
        }


        /*	the current states bits become the next states previous bits  */
        lastEncoded = encoded;


    }


#endif