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
// #include <robotConstants.h>
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
robot::RobotSubSystem lazySusanSystem(LAZY_SUSAN_LIMIT_SWITCH, -1, &lazySusanMotor, 2.6, 0.68, 1.8, -0.7, 160);//-1.0);
//backwards is rotation towards limit switch. coupled with increasing encoder value
//tuning will need to have loop gain < 0.
//forward 172
//left 0
//right 340
//reverse: 536

encoder::RotaryEncoder linearArmEncoder(LINEAR_ARM_ROTARY_ENCODER_PB, LINEAR_ARM_ROTARY_ENCODER_PA);
movement::EncodedMotor linearArmMotor(LINEAR_ARM_P2, LINEAR_ARM_P1, &linearArmEncoder);//
robot::RobotSubSystem linearArmSystem(LINEAR_ARM_LIMIT_SWITCH_A, LINEAR_ARM_LIMIT_SWITCH_B, &linearArmMotor,
1.8, 0.7, 0.7, -0.8, 190.0);//0.8); //95!!!
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
    PROCESS_STATION_PLATE, 
    TRANSITION_TO_SERVE,
    PROCESS_STATION_SERVE,
    IDLE,
};
State currentState = PROCESS_STATION_PLATE; 
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
    clawServo.attach(CLAW_SERVO_PIN, 500, 2400);
    forkliftServo.attach(FORKLIFT_SERVO_PIN, 500, 2400);
    // pinMode(CLAW_SERVO_PIN, OUTPUT);
    // pinMode(FORKLIFT_SERVO_PIN, OUTPUT);

    // myServo.attach(25, 500, 2400);
    // myServo2.attach(26, 500, 2400);

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

    lazySusanSystem.localize(200, 200);
    delay(2000);
}

int updatePIDCount = 0;

void loop() {
    switch (currentState) {
    case PROCESS_STATION_PLATE: {
        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt(); 
            if(receivedVal == 1) { //waits for bp to finish driving
               //move lazy susan to plate side
                while(abs(lazySusanEncoder.getIncrements()-NINETY_LAZYSUSAN) >= ERROR_THRESHOLD){
                lazySusanSystem.updatePID(NINETY_LAZYSUSAN);
                }
        //move arm out
        while (digitalRead(LINEAR_ARM_LIMIT_SWITCH_A)){
            linearArmMotor.backward(200);
            }
            SerialPort.println(1);  // yo bluepill move up the elevator 
            }
        }
        
         if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt(); 
            if(receivedVal == 2) { //bluepill said i moved the elevator
                while (digitalRead(LINEAR_ARM_LIMIT_SWITCH_B)){
                    //arm retracted
                    linearArmMotor.forward(200); 
            }
            SerialPort.println(3); //tell bluepill to drive to serving station
            }
        }
        if(SerialPort.available()){
            int receivedVal = SerialPort.parseInt();
            if(receivedVal == 1) {
                currentState = PROCESS_STATION_SERVE;
            }
        }
    }
    break;

    case PROCESS_STATION_SERVE: {
        //turn towards serving station
        while(abs(lazySusanEncoder.getIncrements()-TWO_SEVENTY_LAZYSUSAN) >= ERROR_THRESHOLD){
                lazySusanSystem.updatePID(TWO_SEVENTY_LAZYSUSAN);
            }
            //move arm out 
            while (digitalRead(LINEAR_ARM_LIMIT_SWITCH_A)){
                linearArmMotor.backward(200);
            }
            Serial.println(1); //tell elevator move down elevator

            if (SerialPort.available()){
                int recieved = SerialPort.parseInt();
                if (recieved == 1){ //elevator moved down
                    while (digitalRead(LINEAR_ARM_LIMIT_SWITCH_B)){
                        linearArmMotor.forward(200); //retract the claw
                        }
                    SerialPort.println(3); //tell elevator state finished
                    currentState = IDLE;
                }
            }
        } break;
    default: {
            currentState = IDLE;
        } break;
    }
}
 //loop

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