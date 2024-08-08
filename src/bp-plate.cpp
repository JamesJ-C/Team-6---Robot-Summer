
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

void isrUpdateLineCount();

/*  Object declerations  */

HardwareSerial SerialPort(USART3);

movement::Motor motorL(MOTOR_L_P2, MOTOR_L_P1);
movement::Motor motorR(MOTOR_R_P1, MOTOR_R_P2);

encoder::RotaryEncoder elevatorEncoder(ELEVATOR_ENCODER_PB, ELEVATOR_ENCODER_PA);
movement::EncodedMotor ElevatorMotor(ELEVATOR_P2, ELEVATOR_P1, &elevatorEncoder);

//robot::RobotSubSystem Elevator();
robot::RobotSubSystem ElevatorSystem(ELEVATOR_LIMIT_BOTTOM, ELEVATOR_LIMIT_TOP, &ElevatorMotor, 7.2, 1.3, 2.2, 1.0, 3100);//1.9, 3.0);
//bottom -133
//top 182
//going uip increase encoder, going down decreases encoder
//up is forward
//down is backward

#define ELEVATOR_CLAW_AT_COUNTER_HEIGHT -140
#define FORKLIFT_COUNTER_HEIGHT 0

robot::DrivePID 
driveSystem(TAPE_SENSOR_FORWARD_2, TAPE_SENSOR_FORWARD_1, TAPE_SENSOR_BACKWARD_1, TAPE_SENSOR_BACKWARD_2, &motorL, &motorR); 

enum State{ 
    START, 

    TRANSITION_TO_PLATE,
    PROCESS_STATION_PLATE,

    TRANSITION_TO_SERVE,
    PROCESS_STATION_SERVE,

    IDLE,
    FINISHED,

    LIFT_PLATE,
    PLATE_DOWN

};
State currentState = START;

/*  Tape debouncing vars  */
bool prevLeftState = false;
bool prevRightState = false; 
int leftLineCount = 0;
int rightLineCount = 0; 

bool r_prevVal = 0;

bool l_prevVal = 0;

unsigned long r_lastTime = 0;
unsigned long r_currentTime = 0;

unsigned long l_lastTime = 0;
unsigned long l_currentTime = 0;


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

    pinMode(START_BUTTON, INPUT_PULLUP);
    

    attachInterrupt(digitalPinToInterrupt(elevatorEncoder.getPinA()), isrUpdateElevatorEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(elevatorEncoder.getPinB()), isrUpdateElevatorEncoder, CHANGE);

}

int countTimes = 0;
void loop(){

    switch (currentState){

    case START: {
        delay(1000); 
        currentState = TRANSITION_TO_PLATE;
    } break;

    case TRANSITION_TO_PLATE: {

        while(!stopConditionsMet_TRANS_TO_4()){
            driveSystem.updateForwardDrivePID();
            updateLineCounts();
        }

        motorL.stop();
        motorR.stop(); 

        SerialPort.println(1);

        currentState = PROCESS_STATION_PLATE;
    } break; 

    case PROCESS_STATION_PLATE: {

            if (ELEVATOR_LIMIT_BOTTOM == HIGH){
                ElevatorMotor.backward(3000);
            } else {
                ElevatorMotor.off();
            }

            if( SerialPort.available() ){
                //wait for lazySusan & arm & claw to move
                int received = SerialPort.parseInt();
                if (received == 1){
                    currentState = LIFT_PLATE;
                }
            } 
    } break;

    case LIFT_PLATE: {
            if (ELEVATOR_LIMIT_TOP == HIGH){
                ElevatorMotor.forward(3800);
            } else {
                ElevatorMotor.off();
                Serial.println(1);
                currentState = TRANSITION_TO_SERVE;
            }
    } break;

    case TRANSITION_TO_SERVE: {
        unsigned long serveStartTime = millis();
        while (millis() - serveStartTime < 2000){
            driveSystem.updateBackwardDrivePID();
        }
        motorR.stop();
        motorL.stop();
        
        SerialPort.println(1);
        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt(); 
            if(receivedVal == 2){
                currentState = PROCESS_STATION_SERVE; 
            }
        }
    } break;

    case PROCESS_STATION_SERVE: { 

    //wait for esp to move ls & arm
        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt();
            if(receivedVal = 1){
                currentState = PLATE_DOWN; 
            }
        } 
    } break;

    case PLATE_DOWN: {

        if (digitalRead(ELEVATOR_LIMIT_BOTTOM == HIGH)){
            ElevatorMotor.backward(2800);
        } else{
            ElevatorMotor.off();
            SerialPort.println(1);
        }

    } break;

case FINISHED: {
        
    } break;

    }

}

void updateLineCounts(){
    
    int l_val = analogRead(TAPE_SENSOR_LEFT_1);
    int r_val = analogRead(TAPE_SENSOR_RIGHT_1);

    int r_tape_val = 0;
    int l_tape_val = 0;
    if ( abs( l_val - r_val ) <= 175) {
        r_tape_val = 0;
    }
    else {
        r_tape_val = r_val;
        l_tape_val = l_val;
    }

    bool r_t_val = r_tape_val >= 700 ? 1 : 0;//maybe 750 is best
    bool l_t_val = l_tape_val >= 700 ? 1 : 0;//maybe 750 is best

//update right lines
    if (r_t_val != r_prevVal && r_t_val != 0){

        r_currentTime = millis();
        if (r_currentTime - r_lastTime >= 1000){
            rightLineCount++;
            motorR.stop();
            motorL.stop();
            r_lastTime = r_currentTime;
        }
    }
//update left
    if (l_t_val != l_prevVal && l_t_val != 0){

        l_currentTime = millis();
        if (l_currentTime - l_lastTime >= 1000){
            leftLineCount++;
            motorR.stop();
            motorL.stop();
            //delay(3000);
            l_lastTime = l_currentTime;
        }
    }
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