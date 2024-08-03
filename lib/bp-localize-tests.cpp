
// #ifndef ESP32

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
bool stopConditionsMet();
int lineCountLeft();
int lineCountRight();
bool markerDetected();

bool stopConditionsMet_TRANS_TO_4();
bool stopConditionsMet_TRANS_TO_5();
bool stopConditionsMet();


/*  Object declerations  */

HardwareSerial SerialPort(USART3);

movement::Motor motorL(MOTOR_L_P1, MOTOR_L_P2);
movement::Motor motorR(MOTOR_R_P1, MOTOR_R_P2);

encoder::RotaryEncoder elevatorEncoder(ELEVATOR_ENCODER_PA, ELEVATOR_ENCODER_PB);
movement::EncodedMotor ElevatorMotor(ELEVATOR_P2, ELEVATOR_P1, &elevatorEncoder);

//robot::RobotSubSystem Elevator();
robot::RobotSubSystem ElevatorSystem(ELEVATOR_LIMIT_BOTTOM, ELEVATOR_LIMIT_TOP, &ElevatorMotor);

robot::DrivePID 
driveSystem(TAPE_SENSOR_FORWARD_2, TAPE_SENSOR_FORWARD_1, TAPE_SENSOR_BACKWARD_1, TAPE_SENSOR_BACKWARD_2, &motorL, &motorR); 



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


void setup() {

    delay(2000);

    //Serial.begin(115200);
    //Serial.println("setup");
    SerialPort.begin(115200);


    pinMode(elevatorEncoder.getPinA(), INPUT_PULLUP);
    pinMode(elevatorEncoder.getPinB(), INPUT_PULLUP);

    pinMode(motorL.getPinA(), OUTPUT);
    pinMode(motorL.getPinB(), OUTPUT);

    pinMode(motorR.getPinA(), OUTPUT);
    pinMode(motorR.getPinB(), OUTPUT);

    pinMode(ElevatorMotor.getPinA(), OUTPUT);
    pinMode(ElevatorMotor.getPinB(), OUTPUT);

    pinMode(ElevatorSystem.getLimit1(), INPUT);
    pinMode(ElevatorSystem.getLimit2(), INPUT);
    

    attachInterrupt(digitalPinToInterrupt(elevatorEncoder.getPinA()), isrUpdateElevatorEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(elevatorEncoder.getPinB()), isrUpdateElevatorEncoder, CHANGE);



    ElevatorSystem.localize(3400, 2500);

    while (true){ 
        if ( SerialPort.available()){
            if (SerialPort.parseInt() == 1){
                SerialPort.println(1);
                break;
            }
        }
    }

}


void loop(){

    // ElevatorSystem.updatePID(10);
    // ElevatorMotor.forward(3800);
    // delay(1000);
    // SerialPort.println("enc val: " + String ( ElevatorSystem.motor->encoder->getIncrements() ));
    // ElevatorMotor.backward(3000);
    // delay(500);
    // SerialPort.println("enc val: " + String ( ElevatorSystem.motor->encoder->getIncrements() ));




    // if ( digitalRead(ELEVATOR_LIMIT_BOTTOM) == HIGH){
    //         SerialPort.println("bottom pushed");
    //         ElevatorMotor.forward(3400);
    //         //delay(800);
    // }

    // if ( digitalRead(ELEVATOR_LIMIT_TOP) == HIGH){
    //     SerialPort.println("top pushed");
    //     ElevatorMotor.backward(2900);
    //     //delay(800);
    // }
    


}
bool markerDetected(){
    return (analogRead(TAPE_SENSOR_LEFT_1) >= TAPE_THRESHOLD || analogRead(TAPE_SENSOR_RIGHT_1) >= TAPE_THRESHOLD);
}

bool prevLeftState = false;
bool prevRightState = false; 
int leftLineCount = 0;
int rightLineCount = 0; 

void updateLineCounts(){
    bool currentLeftState = analogRead(TAPE_SENSOR_LEFT_1) >= TAPE_THRESHOLD;
    bool currentRightState = analogRead(TAPE_SENSOR_RIGHT_1) >= TAPE_THRESHOLD;

    if(currentLeftState && !prevLeftState){
        leftLineCount++;
    }
    if(currentRightState && !prevRightState){
        rightLineCount++;
    }
    prevLeftState = currentLeftState;
    prevRightState = currentRightState;
}

int lineCountRight(){
    return rightLineCount; 
}

int lineCountLeft(){
    return leftLineCount; 
}

bool stopConditionsMet_TRANS_TO_4() {
    return lineCountRight() >= 2 || lineCountLeft() >= 4; 
}

bool stopConditionsMet_TRANS_TO_5() {
    return lineCountRight() >= 1 || lineCountLeft() >= 2; 
}

bool stopConditionsMet() {
    return lineCountRight() >= 1 || lineCountLeft() >= 2; 
}

void isrUpdateElevatorEncoder(){

    bool A = digitalRead(elevatorEncoder.getPinA());
    bool B = digitalRead(elevatorEncoder.getPinA());
    elevatorEncoder.updateEncoder(A, B);

}

// #endif