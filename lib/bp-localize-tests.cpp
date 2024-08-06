
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

    Serial.begin(115200);
    Serial.println("setup");

    SerialPort.begin(115200);


    pinMode(elevatorEncoder.getPinA(), INPUT_PULLUP);
    pinMode(elevatorEncoder.getPinB(), INPUT_PULLUP);

    pinMode(motorL.getPinA(), OUTPUT);
    pinMode(motorL.getPinB(), OUTPUT);

    pinMode(motorR.getPinA(), OUTPUT);
    pinMode(motorR.getPinB(), OUTPUT);

    // pinMode(ElevatorMotor.getPinA(), OUTPUT);
    // pinMode(ElevatorMotor.getPinB(), OUTPUT);

    pinMode(ElevatorSystem.getLimit1(), INPUT);
    pinMode(ElevatorSystem.getLimit2(), INPUT);
    

    attachInterrupt(digitalPinToInterrupt(elevatorEncoder.getPinA()), isrUpdateElevatorEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(elevatorEncoder.getPinB()), isrUpdateElevatorEncoder, CHANGE);


    // ElevatorSystem.localize(3700, 2500);

    // while (true){ 
    //     if ( SerialPort.available()){
    //         if (SerialPort.parseInt() == 1){
    //             SerialPort.println(1);
    //             break;
    //         }
    //     }
    // }

// Serial.println(elevatorEncoder.getMaxIncrement());
// SerialPort.println(elevatorEncoder.getMaxIncrement());

// delay(1000);

Serial.println("setupp");

}

bool prevLeftState = false;
bool prevRightState = false; 
int leftLineCount = 0;
int rightLineCount = 0; 


int lineCount  = 0;
bool prevVal = 0;

// void loop(){


// {
// int i_val = analogRead(TAPE_SENSOR_RIGHT_1);
// int temp = i_val;

// bool val = i_val >= 350 ? 1 : 0; 

// driveSystem.updateForwardDrivePID();

// if (val != prevVal && val != 0){
//     lineCount++;
//     Serial.println(temp);
//     Serial.println();
//     Serial.println(val);
//     Serial.println();
//     Serial.println(lineCount);    
//     Serial.println();     
//     Serial.println();
// }

// if (lineCount >= 2){
//     Serial.println("motors off");
//     lineCount = 0;
//     motorL.stop();
//     motorR.stop();
//     delay(1000);
//     Serial.println("motors on");
// }

// prevVal = val;


}
{
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

// motorL.forward(3900);
// motorR.forward(3900);

// driveSystem.updateForwardDrivePID();
// driveSystem.updateBackwardDrivePID();

// motorL.backward(3900);
// motorR.backward(3900);

// Serial.print("tp1: " + String( analogRead(TAPE_SENSOR_FORWARD_1) ));
// Serial.print(" ");
// Serial.println(analogRead(TAPE_SENSOR_FORWARD_2));

// {
// switch (currentState){
// case START: 
//     delay(1000); 
//     currentState = TRANSITION_TO_4;
//     break;

// case TRANSITION_TO_4:




// // Serial.print("tp1: " + String( analogRead(TAPE_SENSOR_RIGHT_1) ));
// // Serial.print(" ");
// // Serial.println(analogRead(TAPE_SENSOR_LEFT_1));

// int oldC = 0;

//     do {
//          driveSystem.updateForwardDrivePID();
//          updateLineCounts();
//     // if (rightLineCount != oldC){
//     //     Serial.println(rightLineCount);
//     //     SerialPort.println(rightLineCount);
//     //     }
//     //     oldC = rightLineCount;
//     } while(!stopConditionsMet_TRANS_TO_4());
//     motorL.stop();
//     motorR.stop();
//     lineCount = 0;
//     rightLineCount = 0;
//     leftLineCount = 0;
//     delay(2000);

void loop{
    do{
        driveSystem.updateForwardDrivePID(); 
    }
    while(analogRead(TAPE_SENSOR_LEFT_1) <= 600 && analogRead(TAPE_SENSOR_RIGHT_1) <= 600); 
    motorL.stop();
    motorR.stop(); 
    delay(5000); 
}




//     motorL.stop();
//     motorR.stop(); 

//     SerialPort.println(1); 

//     currentState = PROCESS_STATION_4;
//     break; 

// case PROCESS_STATION_4:
//     if( SerialPort.available() ){
//         int receivedVal = SerialPort.parseInt(); 
//         if(receivedVal == 2){
//             ElevatorSystem.moveToValue(-500); 
//             SerialPort.println(3); 
//         }
//     } 
//     if( SerialPort.available() ){
//         int receivedVal = SerialPort.parseInt(); 
//         if(receivedVal == 4){//arm has moved forward 
//         ElevatorSystem.moveToValue(-550);
//         SerialPort.println(1);
//         }
//     } 
//     currentState = TRANSITION_TO_6;

//     break;

//     default:
//     break;

// }

// }


// SerialPort.print(String (elevatorEncoder.getIncrements()) + ". " + String (ElevatorSystem.updatePID(-100)) );

// SerialPort.println("g: " + String (ElevatorSystem.updatePID(-100)) );

// Serial.println(elevatorEncoder.getIncrements());
// SerialPort.println(ElevatorSystem.updatePID(-100));


}




bool markerDetected(){
    return (analogRead(TAPE_SENSOR_LEFT_1) >= TAPE_THRESHOLD || analogRead(TAPE_SENSOR_RIGHT_1) >= TAPE_THRESHOLD);
}


void updateLineCounts(){

    bool currentLeftState = analogRead(TAPE_SENSOR_LEFT_1) >= 600;
    bool currentRightState = analogRead(TAPE_SENSOR_RIGHT_1) >= 600;

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