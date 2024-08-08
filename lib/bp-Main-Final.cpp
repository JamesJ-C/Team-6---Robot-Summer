
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
    PROCESS_STATION_CHEESE,
    PROCESS_STATION_CHEESE_1, 
    PROCESS_STATION_CHEESE_2, 
    PROCESS_STATION_CHEESE_3, 
    PROCESS_STATION_CHEESE_4, 
    PROCESS_STATION_CHEESE_5, 

    TRANSITION_TO_SERVE,
    PROCESS_STATION_SERVE,
    TRANSITION_TO_CHEESE,
    TRANSITION_TO_SERVE2,
    PROCESS_STATION_SERVE_2,
    IDLE,
    FINISHED,
    MOVE_ELEVATOR,
    MOVE_ARM,
    MOVE_ELEVATOR_FORKLIFT,
    MOVE_ELEVATOR_LIFTED,
    MOVE_ELEVATOR_COUNTER, 
    MOVE_ELEVATOR_COUNTER_1,
    MOVE_ELEVATOR_COUNTER_2,
    MOVE_ELEVATOR_COUNTER_3, 
    PROCESS_STATION_PLATE_1,
    PROCESS_STATION_PLATE_2,
    PROCESS_STATION_PLATE_3,
    PROCESS_STATION_PLATE_4,
    MOVE_ELEVATOR_FORKLIFT_1,
    MOVE_ELEVATOR_FORKLIFT_2,
    MOVE_ELEVATOR_FORKLIFT_3,
    MOVE_ELEVATOR_LIFTED_1,
    MOVE_ELEVATOR_LIFTED_2,
    MOVE_ELEVATOR_LIFTED_3,

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


volatile bool STARTED = false;



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

case MOVE_ELEVATOR_COUNTER_1: {

    if ( ElevatorSystem.updatePID(ELEVATOR_CLAW_AT_COUNTER_HEIGHT) == 0){
        countTimes++;
    }
    if (countTimes >= 30){
        currentState = PROCESS_STATION_CHEESE_2;
        SerialPort.println(2);
        countTimes = 0;
    }

} break;


case MOVE_ELEVATOR_LIFTED_1: {
    if ( ElevatorSystem.updatePID(ELEVATOR_CLAW_AT_COUNTER_HEIGHT - 40) == 0){
        countTimes++;
    }
    if (countTimes >= 30){
        currentState = PROCESS_STATION_CHEESE_3;
        SerialPort.println(4);
        countTimes = 0;
    }
} break;

case MOVE_ELEVATOR_LIFTED_2: {
    if ( ElevatorSystem.updatePID(ELEVATOR_CLAW_AT_COUNTER_HEIGHT - 40) == 0){
        countTimes++;
    }
    if (countTimes >= 30){
        currentState = IDLE;
        countTimes = 0;
    }
} break;

case MOVE_ELEVATOR_LIFTED_3: {
    if ( ElevatorSystem.updatePID(ELEVATOR_CLAW_AT_COUNTER_HEIGHT - 40) == 0){
        countTimes++;
    }
    if (countTimes >= 30){
        currentState = PROCESS_STATION_PLATE_3;
        countTimes = 0;
        SerialPort.println(4);
    }
} break;

case MOVE_ELEVATOR_FORKLIFT: {
    if ( ELEVATOR_LIMIT_BOTTOM){
        ElevatorMotor.backward(2700);
    }
    if ( !ELEVATOR_LIMIT_BOTTOM) {
        currentState = PROCESS_STATION_PLATE_3;
        SerialPort.println(4);
    }
} break;

case MOVE_ELEVATOR_FORKLIFT_2: {
    if ( ELEVATOR_LIMIT_BOTTOM){
        ElevatorMotor.backward(2700);
    }
    if ( !ELEVATOR_LIMIT_BOTTOM) {
        currentState = PROCESS_STATION_PLATE_2;
        SerialPort.println(2);
    }
} break;


    case START: {
        delay(1000); 
        currentState = PROCESS_STATION_CHEESE;
    }
        break;

    case TRANSITION_TO_PLATE: {

        while(!stopConditionsMet_TRANS_TO_4()){
            driveSystem.updateForwardDrivePID();
            updateLineCounts();
        }

        motorL.stop();
        motorR.stop(); 

        SerialPort.println(1);

        currentState = PROCESS_STATION_PLATE;
    }
        break; 

    case PROCESS_STATION_PLATE: {
        if( SerialPort.available() ){
            //wait for lazySusan & arm & claw to move
            int received = SerialPort.parseInt();
            if (received == 1){
                currentState = MOVE_ELEVATOR_FORKLIFT_2;
            }
        } 
    } break;

        //move elevator to height just above plate 

        //send done response 

        //esp will retract, move forklift down

        //wait for response to move elevaotr to bottom 

        //send done response 

        //esp will extend arm

    case PROCESS_STATION_PLATE_2: {
        if( SerialPort.available() ){
            //wait for lazySusan & arm & claw to move
            int received = SerialPort.parseInt();
            if (received == 3){
                currentState = MOVE_ELEVATOR_LIFTED_3;
            }
        } 
    } break;

    case PROCESS_STATION_PLATE_3: {
        if( SerialPort.available() ){
            int received = SerialPort.parseInt();
            //wait for response to move to serving station
            if (received == 5){
                currentState = TRANSITION_TO_SERVE;
            }
        }
    }
    break;

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
    }
        break;


    case PROCESS_STATION_SERVE: { 

    //wait for esp to move ls & arm
        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt();
            if(receivedVal = 1){
                currentState = MOVE_ELEVATOR_FORKLIFT_2; 
            }
        } 
    } break;

    case PROCESS_STATION_SERVE_2: {

        //move elevator down

        //tell esp done 

        //esp retracts arms
        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt();
        //waits for signal from esp to move elevator
            if(receivedVal = 3){
            currentState = MOVE_ELEVATOR_LIFTED_2; 
        }
        }
    }
        //go from serving back to cheese station
        break;

    case TRANSITION_TO_CHEESE: {

        while( rightLineCount < 1){
            updateLineCounts();
            driveSystem.updateForwardDrivePID();
            //pidDriving(); //to the left
        }
        motorL.stop();
        motorR.stop(); 
        SerialPort.println(1);

        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt(); 
            if(receivedVal == 2){
                currentState = PROCESS_STATION_CHEESE;
            }
        }
    }
    break;

    case PROCESS_STATION_CHEESE: {
    //wait for lazySusan to turn & arm to come out & claw to open
        if(SerialPort.available()) {
            int receivedVal = SerialPort.parseInt();

            if(receivedVal == 1){
                currentState = MOVE_ELEVATOR_COUNTER_1;
            } 
        }
    } break;


    case PROCESS_STATION_CHEESE_2: {        

            SerialPort.println(2);
            int receivedVal = SerialPort.parseInt();
            if(receivedVal == 3){
                //move elevator up
                currentState = MOVE_ELEVATOR_LIFTED_1;
            }

    } break;

    case PROCESS_STATION_CHEESE_3: {

        if(SerialPort.available()) {
            int receivedVal = SerialPort.parseInt();
            if(receivedVal == 5){
                currentState = TRANSITION_TO_PLATE;
            }
        }
    }
    break;

    case TRANSITION_TO_SERVE2: {
        
        
        driveSystem.updateForwardDrivePID();
        //pidDriving(); //to the right
        updateLineCounts();//run in drving while loop 
        //stopping at Serving area; 
        //once stopped Serial.println(1);
        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt(); 
            if(receivedVal == 6)
            currentState = FINISHED; 
        }
    }
        break;
    case FINISHED: {//aka transition to 4.2 
        
        while(!stopConditionsMet()){
            //pidDriving();
            updateLineCounts();
            driveSystem.updateForwardDrivePID(); 
        }
        motorL.stop();
        motorR.stop(); 
        SerialPort.println(1);
        currentState = PROCESS_STATION_PLATE; 
    }
        break;
        
        default:
            break;
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