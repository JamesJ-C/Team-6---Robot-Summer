// #ifdef ESP32


#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

void isrUpdateRetractArmButton();
void isrUpdateExtendArmButton();



// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


encoder::RotaryEncoder lazySusanEncoder(LAZY_SUSAN_ROTARY_ENCODER_PB, LAZY_SUSAN_ROTARY_ENCODER_PA);
movement::EncodedMotor lazySusanMotor(LAZY_SUSAN_P1, LAZY_SUSAN_P2, &lazySusanEncoder);
robot::RobotSubSystem lazySusanSystem(LAZY_SUSAN_LIMIT_SWITCH, -1, &lazySusanMotor, 2.6, 0.68, 1.8, -1.0, 160);//-1.0);
//backwards is rotation towards limit switch. coupled with increasing encoder value
//tuning will need to have loop gain < 0.
//forward 172
//left 0
//right 340
//reverse: 536



encoder::RotaryEncoder linearArmEncoder(LINEAR_ARM_ROTARY_ENCODER_PB, LINEAR_ARM_ROTARY_ENCODER_PA);
movement::EncodedMotor linearArmMotor(LINEAR_ARM_P2, LINEAR_ARM_P1, &linearArmEncoder);//
robot::RobotSubSystem linearArmSystem(LINEAR_ARM_LIMIT_SWITCH_A, LINEAR_ARM_LIMIT_SWITCH_B, &linearArmMotor,
1.8, 0.7, 1.8, -0.8, 95.0);//0.8);
//forward is positive encoder values: -70 to 28, range of 98 ~100
//extended ~100
//retracted 0

robot::IRSensor beaconSensor(IR_SENSOR_1, IR_SENSOR_2);


Servo clawServo;
Servo forkliftServo;
clawActuation::Claw clawSystem(&clawServo, &forkliftServo, CLAW_LIMIT_SWITCH_A, CLAW_LIMIT_SWITCH_B);


// Servo clawServo;
// Servo forkliftServo;
// clawActuation::Claw clawSystem(&clawServo, &forkliftServo, CLAW_LIMIT_SWITCH_A, CLAW_LIMIT_SWITCH_B);

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
    MOVE_ARM,
    CLAW,
    START_1, 
    MOVE_LS
};
State currentState = START_1; 

enum uartSignal {
    sendingSignal,
    waitingForReception,
    finish
};

uartSignal sig = sendingSignal;

HardwareSerial SerialPort(1);

void setup() {

    delay(2000);

    /*  Serial setup  */
    Serial.begin(115200);
    Serial.println("Setup");
    SerialPort.begin(115200, SERIAL_8N1, RX, TX);

/*  Display setup  */

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(2000);

  // Displays "Hello world!" on the screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Setting up...");
  display.display();

    /*  Servos  */
    clawServo.attach(CLAW_SERVO_PIN);
    forkliftServo.attach(FORKLIFT_SERVO_PIN);
    pinMode(CLAW_SERVO_PIN, OUTPUT);
    pinMode(FORKLIFT_SERVO_PIN, OUTPUT);

    /*  Encoders  */
    pinMode(lazySusanEncoder.getPinA(), INPUT_PULLUP);
    pinMode(lazySusanEncoder.getPinB(), INPUT_PULLUP);

    pinMode(linearArmEncoder.getPinA(), INPUT_PULLUP);
    pinMode(linearArmEncoder.getPinB(), INPUT_PULLUP);

    /*  Motors  */
    pinMode(lazySusanMotor.getPinA(), OUTPUT);
    pinMode(lazySusanMotor.getPinB(), OUTPUT);

    pinMode(linearArmMotor.getPinA(), OUTPUT);
    pinMode(linearArmMotor.getPinB(), OUTPUT);


    pinMode(linearArmSystem.getLimit1(), INPUT);
    pinMode(linearArmSystem.getLimit2(), INPUT);

    pinMode(37, INPUT_PULLUP);


    /*  Interrupts  */
    attachInterrupt(lazySusanEncoder.getPinA(), isrUpdateLazySusanEncoder, CHANGE);
    attachInterrupt(lazySusanEncoder.getPinB(), isrUpdateLazySusanEncoder, CHANGE);

    attachInterrupt(digitalPinToInterrupt(linearArmEncoder.getPinA()), isrUpdateLinearArmEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(linearArmEncoder.getPinB()), isrUpdateLinearArmEncoder, CHANGE);


    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("localizing...");
    display.display();


    Serial.println("localizing");

    
    lazySusanSystem.localize(200, 200);
    delay(2000);
    linearArmSystem.localize(150, 150);

    // display.clearDisplay();
    // display.setTextSize(1);
    // display.setTextColor(SSD1306_WHITE);
    // display.setCursor(0,0);
    // display.println(linearArmEncoder.getMaxIncrement());
    // display.display();

    while (abs (linearArmSystem.updatePID(30)) <= 30){

    }
    linearArmMotor.off();
    lazySusanMotor.off();
    //delay(2000);

        display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0,0);
            display.print("finished lcze");                
            display.display();

    // SerialPort.println(1);
    unsigned long startTime = millis();
    while (true){
        if ( SerialPort.parseInt() == 1){
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0,0);
            display.print("confirmation complete");                
            display.display();
            delay(1000);
            break;
        }
        else if (SerialPort.parseInt() == 2){
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0,0);
            display.print("recived 2");                
            display.display();
            delay(1000);
        }
        else if (SerialPort.parseInt() == 3){
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(SSD1306_WHITE);
            display.setCursor(0,0);
            display.print("recived 3");                
            display.display();
            delay(1000);
        }
        if (millis() - startTime > 3000 && digitalRead(LAZY_SUSAN_LIMIT_SWITCH) == LOW ){
            SerialPort.println(1);
            startTime = millis();
        }
    }

}

int val = 0;
int loopCount = 0;
int g = 0;
void loop() {


    switch (currentState)
    {
    case START:
        forkliftServo.write(FORKLIFTSERVO_READY_POS);
        if(SerialPort.available()){
        int receivedVal = SerialPort.parseInt();
            if(receivedVal == 1) { // 1 is the signal indicating that the BP has finished driving 
                currentState = PROCESS_STATION_4; 
            }
        }
        break;

    case PROCESS_STATION_4:
        while(abs(lazySusanEncoder.getIncrements()-NINETY_LAZYSUSAN) >= ERROR_THRESHOLD){
            lazySusanSystem.updatePID(NINETY_LAZYSUSAN);
        }
        SerialPort.println(2); 
        //wait for bp to adjust height 
        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt();
            if(receivedVal == 3) {
            while(abs(linearArmEncoder.getIncrements()-CLAW_FORWARD) >= ERROR_THRESHOLD){
                linearArmSystem.updatePID(CLAW_FORWARD);
            }
                Serial.println(4); 
            }
        }
        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt();
            if(receivedVal == 1) {
                currentState = PROCESS_STATION_6;
            }
        }
        break;

        case PROCESS_STATION_6:
        while(abs(lazySusanEncoder.getIncrements()-TWO_SEVENTY_LAZYSUSAN) >= ERROR_THRESHOLD){
                lazySusanSystem.updatePID(TWO_SEVENTY_LAZYSUSAN);
        }
        Serial.println(2);
        if(SerialPort.available()) {
            int receivedVal = SerialPort.parseInt(); 
            if(receivedVal == 3){
                while(abs(linearArmEncoder.getIncrements()-CLAW_NEUTRAL)>= ERROR_THRESHOLD){
                    linearArmSystem.updatePID(CLAW_NEUTRAL);
                }
                Serial.println(4);
            }
        }
        if( SerialPort.available() ) {
            int receivedVal = SerialPort.parseInt(); 
            if( receivedVal == 1 ){
                currentState = PROCESS_STATION_5; 
            }
        }
        break;

        case PROCESS_STATION_5:
        while(abs(lazySusanEncoder.getIncrements()-TWO_SEVENTY_LAZYSUSAN) >= ERROR_THRESHOLD){
            lazySusanSystem.updatePID(TWO_SEVENTY_LAZYSUSAN);
        }
        clawServo.write(CLAWSERVO_OPEN_POS);
        SerialPort.println(2); 
        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt();
            if(receivedVal == 3){
                while(abs(linearArmEncoder.getIncrements()-CLAW_FORWARD) >= ERROR_THRESHOLD){
                    linearArmSystem.updatePID(CLAW_FORWARD);
                }
                for(int pos = CLAWSERVO_OPEN_POS; pos <= CLAWSERVO_CLOSED_POS; pos--){
                    clawServo.write(pos);
                    delay(20);
                }
                SerialPort.println(5); 
            }
        }
        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt(); 
            if(receivedVal == 1){
                currentState = PROCESS_STATION_62;
            }
        }
            break;

        case PROCESS_STATION_62:
            clawServo.write(CLAWSERVO_OPEN_POS); 
            while(abs(linearArmEncoder.getIncrements()-CLAW_NEUTRAL) >= ERROR_THRESHOLD){
                linearArmSystem.updatePID(CLAW_NEUTRAL);
            }
            currentState = FINISHED; 
            SerialPort.println(6);
            break;

        case FINISHED: //basically loops back to station 
            if(SerialPort.available()){
                int receivedVal = SerialPort.parseInt(); 
                if(receivedVal == 1){
                    currentState = PROCESS_STATION_4; 
                }
            }
            break;
        default:
            currentState = IDLE;
        break;

    }    


} //loop

void IRAM_ATTR isrUpdateLinearArmEncoder(){

    // bool A = digitalRead(linearArmEncoder.getPinA());
    // ool B = digitalRead(linearArmEncoder.getPinB());
    // linearArmEncoder.updateEncoder(A, B);
    linearArmEncoder.updateEncoder();
}

void IRAM_ATTR isrUpdateLazySusanEncoder(){

    // bool A = digitalRead(lazySusanEncoder.getPinA());
    // bool B = digitalRead(lazySusanEncoder.getPinB());
    // lazySusanEncoder.updateEncoder(A, B);
    lazySusanEncoder.updateEncoder();

    //val++;
}

void IRAM_ATTR isrUpdateExtendArmButton(){
    linearArmSystem.firstSwitchHit = true;
}

void IRAM_ATTR isrUpdateRetractArmButton(){
    linearArmSystem.secondSwitchHit = true;
}


// #endif