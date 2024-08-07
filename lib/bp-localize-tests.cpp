
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

encoder::RotaryEncoder elevatorEncoder(ELEVATOR_ENCODER_PB, ELEVATOR_ENCODER_PA);
movement::EncodedMotor ElevatorMotor(ELEVATOR_P2, ELEVATOR_P1, &elevatorEncoder);

//robot::RobotSubSystem Elevator();
robot::RobotSubSystem ElevatorSystem(ELEVATOR_LIMIT_BOTTOM, ELEVATOR_LIMIT_TOP, &ElevatorMotor, 9.2, 0.6, 2.1, 8.0, 3200);//1.9, 3.0);
//bottom -133
//top 182
//going uip increase encoder, going down decreases encoder
//up is forward
//down is backward
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
State currentState = START;


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

// delay(40000);

    // ElevatorSystem.localize(3800, 3200);
    // ElevatorSystem.moveToValue(-400);

    // while (true){ 
    //     if ( SerialPort.available()){
    //         if (SerialPort.parseInt() == 1){
    //             SerialPort.println(1);
    //             break;
    //         }
    //     }
    // }

}

bool prevLeftState = false;
bool prevRightState = false; 
int leftLineCount = 0;
int rightLineCount = 0; 


int lineCount  = 0;
bool prevVal = 0;

bool off = false;

unsigned long lastTime = 0;
unsigned long currentTime = 0;

void loop(){


// ElevatorMotor.forward(3800);

    // Serial.println(elevatorEncoder.getIncrements());
    // ElevatorSystem.updatePID(235);


// if(digitalRead(ELEVATOR_LIMIT_BOTTOM) == HIGH){
//     ElevatorMotor.forward(3700);
//     // Serial.print("fwd: ");
//     // Serial.println(elevatorEncoder.getIncrements());                 
// }

// if(digitalRead(ELEVATOR_LIMIT_TOP) == HIGH){
//     ElevatorMotor.backward(3700);
//     // Serial.print("backwd: ");
//     // Serial.println(elevatorEncoder.getIncrements());   

// }
    // Serial.println(elevatorEncoder.getIncrements());             



    //ElevatorSystem.moveToValue(-250);//  updatePID(250);
    //SerialPort.println(2);




// switch (currentState){
    
// case START: 
//     //delay(1000); 
//     currentState = MOVE_ELEVATOR;
//     break;

// case MOVE_ELEVATOR:

//     if ( abs(ElevatorSystem.updatePID(250) ) <= 30) {
//         SerialPort.println(2);
//         off = true;
//         currentState = IDLE;
//         ElevatorMotor.off();
//     }
//     break;
// case IDLE:
//     ElevatorMotor.off();
//     default:
//     break;

// }


// SerialPort.print(String (elevatorEncoder.getIncrements()) + ". " + String (ElevatorSystem.updatePID(-100)) );

// SerialPort.println("g: " + String (ElevatorSystem.updatePID(-100)) );

// Serial.println(elevatorEncoder.getIncrements());
// SerialPort.println(ElevatorSystem.updatePID(-100));

// }



    int l_val = analogRead(TAPE_SENSOR_LEFT_1);
    int r_val = analogRead(TAPE_SENSOR_RIGHT_1);

    int tape_val = 0;
    if ( abs( l_val - r_val ) <= 175) {
        tape_val = 0;
    }
    else {
        tape_val = r_val;
    }

    bool val = tape_val >= 700 ? 1 : 0;//maybe 750 is best

    driveSystem.updateForwardDrivePID();

    if (val != prevVal && val != 0){

        currentTime = millis();
        if (currentTime - lastTime >= 1000){
            lineCount++;
            motorR.stop();
            motorL.stop();
            //delay(3000);
            lastTime = currentTime;
        }
    }

    if (lineCount >= 2){
        // Serial.println("motors off");
        lineCount = 0;
        motorL.stop();
        motorR.stop();
        delay(1000);
        // Serial.println("motors on");
    }


}


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