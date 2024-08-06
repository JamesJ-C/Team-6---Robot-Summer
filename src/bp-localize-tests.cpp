
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
bool stopConditionsMet();
int lineCountLeft();
int lineCountRight();
bool markerDetected();
void updateLineCounts();

bool stopConditionsMet_TRANS_TO_4();
bool stopConditionsMet_TRANS_TO_5();
bool stopConditionsMet();

void isrUpdateLineCount();


/*  Object declerations  */

HardwareSerial SerialPort(USART3);

movement::Motor motorL(MOTOR_L_P2, MOTOR_L_P1);
movement::Motor motorR(MOTOR_R_P1, MOTOR_R_P2);

encoder::RotaryEncoder elevatorEncoder(ELEVATOR_ENCODER_PA, ELEVATOR_ENCODER_PB);
movement::EncodedMotor ElevatorMotor(ELEVATOR_P2, ELEVATOR_P1, &elevatorEncoder);

//robot::RobotSubSystem Elevator();
robot::RobotSubSystem ElevatorSystem(ELEVATOR_LIMIT_BOTTOM, ELEVATOR_LIMIT_TOP, &ElevatorMotor, 6.2, 0.5, 2.1, 1.0);

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
    FINISHED,
    MOVE_ELEVATOR,
    MOVE_ARM
};
State currentState = IDLE;


void setup() {

    delay(2000);

    // Serial.begin(115200);
    // Serial.println("setup");

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


    pinMode(TAPE_SENSOR_FORWARD_1, INPUT);
    pinMode(TAPE_SENSOR_FORWARD_2, INPUT);

    pinMode(TAPE_SENSOR_BACKWARD_1, INPUT);
    pinMode(TAPE_SENSOR_BACKWARD_2, INPUT);

    pinMode(TAPE_SENSOR_RIGHT_1, INPUT);
    pinMode(TAPE_SENSOR_LEFT_1, INPUT);
    
    pinMode(TAPE_SENSOR_RIGHT_2, INPUT);
    pinMode(TAPE_SENSOR_LEFT_2, INPUT);
    

    attachInterrupt(digitalPinToInterrupt(elevatorEncoder.getPinA()), isrUpdateElevatorEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(elevatorEncoder.getPinB()), isrUpdateElevatorEncoder, CHANGE);


    ElevatorSystem.localize(3700, 2500);

    while (true){ 
        if ( SerialPort.available()){
            if (SerialPort.parseInt() == 1){
                SerialPort.println(1);
                break;
            }
        }
    }

}

bool prevLeftState = false;
bool prevRightState = false; 
int leftLineCount = 0;
int rightLineCount = 0; 


int lineCount  = 0;
bool prevVal = 0;

void loop(){

switch (currentState){
    
case START: 
    delay(1000); 
    currentState = MOVE_ELEVATOR;
    break;

case MOVE_ELEVATOR:
    if (SerialPort.available()){
        if (SerialPort.parseInt() == 1){
            ElevatorSystem.moveToValue(270);//  updatePID(250);
            SerialPort.println(2);
            currentState = IDLE;
        }
    }

    default:
    break;

}

}


// SerialPort.print(String (elevatorEncoder.getIncrements()) + ". " + String (ElevatorSystem.updatePID(-100)) );

// SerialPort.println("g: " + String (ElevatorSystem.updatePID(-100)) );

// Serial.println(elevatorEncoder.getIncrements());
// SerialPort.println(ElevatorSystem.updatePID(-100));





bool markerDetected(){
    return (analogRead(TAPE_SENSOR_LEFT_1) >= TAPE_THRESHOLD || analogRead(TAPE_SENSOR_RIGHT_1) >= TAPE_THRESHOLD);
}



void updateLineCounts(){
    bool currentLeftState = analogRead(TAPE_SENSOR_LEFT_1) >= 200;
    bool currentRightState = analogRead(TAPE_SENSOR_RIGHT_1) >= 200;

    // if(currentLeftState && !prevLeftState){
    //     leftLineCount++;
    // }
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


/**
 * @brief returns true if the right line counts are >= 2
 *  or if left line counts are greater than 4
 * 
 * @return true 
 * @return false 
 */
bool stopConditionsMet_TRANS_TO_4() {
    return lineCountRight() >= 2;// || lineCountLeft() >= 4; 
}


/**
 * @brief returns true if the right line counts are >= 1
 *  or if left line counts are >= 2
 * 
 * @return true 
 * @return false 
 */
bool stopConditionsMet_TRANS_TO_5() {
    return lineCountRight() >= 1 || lineCountLeft() >= 2; 
}


/**
 * @brief returns true if the right line counts are >= 1
 *  or if left line counts are >= 2
 * 
 * @return true 
 * @return false 
 */
bool stopConditionsMet() {
    return lineCountRight() >= 1 || lineCountLeft() >= 2; 
}

void isrUpdateElevatorEncoder(){

    // bool A = digitalRead(elevatorEncoder.getPinA());
    // bool B = digitalRead(elevatorEncoder.getPinA());
    // elevatorEncoder.updateEncoder(A, B);

    elevatorEncoder.updateEncoder();

}

#endif