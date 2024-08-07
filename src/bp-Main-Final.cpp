
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

#define ELEVATOR_CLAW_AT_COUNTER_HEIGHT 250
#define FORKLIFT_COUNTER_HEIGHT 0 



robot::DrivePID 
driveSystem(TAPE_SENSOR_FORWARD_2, TAPE_SENSOR_FORWARD_1, TAPE_SENSOR_BACKWARD_1, TAPE_SENSOR_BACKWARD_2, &motorL, &motorR); 


enum State{
    START, 
    TRANSITION_TO_CHEESE,
    PROCESS_STATION_CHEESE, 
    TRANSITION_TO_PLATE,
    PROCESS_STATION_PLATE,
    TRANSITION_TO_SERVE,
    PROCESS_STATION_SERVE,
    FINISHED,
    MOVE_ELEVATOR,
    MOVE_ARM
    IDLE,
};
State currentState = START;

enum PlateStation {
    IDLE,
    st1,
    st2
};

PlateStation currentPlateState = IDLE;



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


void isrStart(){
    STARTED = true;
}

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


    attachInterrupt(digitalPinToInterrupt(START_BUTTON), isrStart, FALLING);


    delay(10000);
    ElevatorSystem.localize(4000, 3500);

    while (true){

            if ( SerialPort.available()){
                if (SerialPort.parseInt() == 1){
                    SerialPort.println(1);
                    SerialPort.flush();
                    break;
                }
            }
    }

}

void loop(){
    switch (currentState){
    case START: 
        delay(1000); 
        currentState = TRANSITION_TO_CHEESE; 
        break;

    case TRANSITION_TO_CHEESE:
    while(rightLineCount < 1){
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
    break;

    case PROCESS_STATION_CHEESE:
    //wait for lazySusan to turn & arm to come out & claw to open

        while(abs(elevatorEncoder.getIncrements()-ELEVATOR_CLAW_AT_COUNTER_HEIGHT) >= ERROR_THRESHOLD){
            ElevatorSystem.updatePID(ELEVATOR_CLAW_AT_COUNTER_HEIGHT);
        }
        SerialPort.println(3);


        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt(); 
            if(receivedVal == 5){
                //move elevator up
                while(abs(elevatorEncoder.getIncrements()- (ELEVATOR_CLAW_AT_COUNTER_HEIGHT + 20) ) >= ERROR_THRESHOLD){
                    ElevatorSystem.updatePID( (ELEVATOR_CLAW_AT_COUNTER_HEIGHT + 20) );
                }

            //wait for esp to tell us to move
                currentState = TRANSITION_TO_PLATE;
            }
        }

        break;

    case TRANSITION_TO_PLATE:
        while(!stopConditionsMet_TRANS_TO_4()){
            driveSystem.updateForwardDrivePID();
            updateLineCounts();
        }

        motorL.stop();
        motorR.stop(); 

        SerialPort.println(1);

        currentState = PROCESS_STATION_PLATE;
        break; 

    case PROCESS_STATION_PLATE:
    if( SerialPort.available() ){
        //wait for lazySusan & arm to move 

        //move elevator to height just above plate 

        //send done response 

        //esp will retract, move forklift down

        //wait for response to move elevaotr to bottom 

        //send done response 

        //esp will extend arm

        //wait for response to move elevator up 
        //send done response 

        //esp retracts

        //wait for response to move to serving station

        int receivedVal = SerialPort.parseInt(); 
        if(receivedVal == 2){
            while(abs(elevatorEncoder.getIncrements()-FORKLIFT_COUNTER_HEIGHT) >= ERROR_THRESHOLD){
                ElevatorSystem.updatePID(FORKLIFT_COUNTER_HEIGHT);
            }
            SerialPort.println(3); 
        }
    } 
    if(SerialPort.available() ){
        int receivedVal = SerialPort.parseInt(); 
        if(receivedVal == 4){
            while(abs(elevatorEncoder.getIncrements()-( FORKLIFT_COUNTER_HEIGHT + 20) ) >= ERROR_THRESHOLD){
                ElevatorSystem.updatePID( ( FORKLIFT_COUNTER_HEIGHT + 20) );
            }

        }
    } currentState = TRANSITION_TO_SERVE; 
        // Serial.println("enc: " + String(    elevatorEncoder.getIncrements() ) );
        // // ElevatorSystem.updatePID(80);
        break;

    case TRANSITION_TO_SERVE:
        unsigned long serveStartTime = millis();
        while (millis() - serveStartTime < 2000){
            driveSystem.updateBackwardDrivePID();
        }
        motorR.stop();
        motorL.stop();
        
        //once stopped Serial.println(1);
        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt(); 
            if(receivedVal == 2){
                currentState = PROCESS_STATION_SERVE; 
            }
        }

        break;


    case PROCESS_STATION_SERVE:
    
//wait for esp to move ls & arm

//move elevator down

//tell esp done 

//esp retracts arms

//waits for signal from esp to move elevator

//go from serving back to cheese station

        while(abs(elevatorEncoder.getIncrements()-FORKLIFT_COUNTER_HEIGHT) >= ERROR_THRESHOLD){
            ElevatorSystem.updatePID(FORKLIFT_COUNTER_HEIGHT);
        }
        Serial.println(3);
        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt();
            if(receivedVal = 4){
                currentState = TRANSITION_TO_CHEESE; 
            }
        }

        break;
    
    case FINISHED: //aka transition to 4.2 
        
        while(!stopConditionsMet()){
            //pidDriving();
            updateLineCounts();
            driveSystem.updateForwardDrivePID(); 
        }
        motorL.stop();
        motorR.stop(); 
        SerialPort.println(1);
        currentState = PROCESS_STATION_PLATE; 
        
        break;
    }

}


bool markerDetected(){
    return (analogRead(TAPE_SENSOR_LEFT_1) >= TAPE_THRESHOLD || analogRead(TAPE_SENSOR_RIGHT_1) >= TAPE_THRESHOLD);
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
