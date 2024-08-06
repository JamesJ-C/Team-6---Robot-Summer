#ifdef ESP32


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
// #include <ESp32Servo.h>
// #include <claw.h>

//#ifdef ESP32

//#endif

void isrUpdateLinearArmEncoder();
void isrUpdateLazySusanEncoder();

void isrUpdateRetractArmButton();
void isrUpdateExtendArmButton();



// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


encoder::RotaryEncoder lazySusanEncoder(LAZY_SUSAN_ROTARY_ENCODER_PA, LAZY_SUSAN_ROTARY_ENCODER_PB);
movement::EncodedMotor lazySusanMotor(LAZY_SUSAN_P1, LAZY_SUSAN_P2, &lazySusanEncoder);
robot::RobotSubSystem lazySusanSystem(LAZY_SUSAN_LIMIT_SWITCH, -1, &lazySusanMotor);

encoder::RotaryEncoder linearArmEncoder(LINEAR_ARM_ROTARY_ENCODER_PA, LINEAR_ARM_ROTARY_ENCODER_PB);
movement::EncodedMotor linearArmMotor(LINEAR_ARM_P1, LINEAR_ARM_P2, &linearArmEncoder);//
robot::RobotSubSystem linearArmSystem(LINEAR_ARM_LIMIT_SWITCH_A, LINEAR_ARM_LIMIT_SWITCH_B, &linearArmMotor,
1.2, 0.6, 1.8, 0.8);

robot::IRSensor beaconSensor(IR_SENSOR_1, IR_SENSOR_2);


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
    FINISHED
};
State currentState = START; 

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
    // clawServo.attach(CLAW_SERVO_PIN);
    // forkliftServo.attach(FORKLIFT_SERVO_PIN);

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


    pinMode(linearArmSystem.getLimit1(), INPUT);
    pinMode(linearArmSystem.getLimit2(), INPUT);


    /*  Interrupts  */
    attachInterrupt(lazySusanEncoder.getPinA(), isrUpdateLazySusanEncoder, CHANGE);
    attachInterrupt(lazySusanEncoder.getPinB(), isrUpdateLazySusanEncoder, CHANGE);

    attachInterrupt(digitalPinToInterrupt(linearArmEncoder.getPinA()), isrUpdateLinearArmEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(linearArmEncoder.getPinB()), isrUpdateLinearArmEncoder, CHANGE);

    // attachInterrupt(digitalPinToInterrupt(linearArmSystem.getLimit1() ), isrUpdateExtendArmButton, RISING);    
    // attachInterrupt(digitalPinToInterrupt(linearArmSystem.getLimit2() ), isrUpdateRetractArmButton, RISING);

    

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println("localizing...");
    display.display();


    Serial.println("localizing");

    // linearArmSystem.localize(125, 125);
    // lazySusanSystem.localize(190, 190);

    // display.clearDisplay();
    // display.setTextSize(1);
    // display.setTextColor(SSD1306_WHITE);
    // display.setCursor(0,0);
    // display.print("done localize. Max: ");
    // display.println(lazySusanEncoder.getMaxIncrement());
    // display.display();

    // linearArmMotor.off();
    // lazySusanMotor.off();

    // delay(3000);
    
    // SerialPort.println(1);
    // unsigned long startTime = millis();
    // while (true){ 
    //     if ( SerialPort.parseInt() == 1){
    //         display.clearDisplay();
    //         display.setTextSize(1);
    //         display.setTextColor(SSD1306_WHITE);
    //         display.setCursor(0,0);
    //         display.print("confirmation complete");                
    //         display.display();
    //         // delay(1000);
    //         break;
    //     }
    //     if (millis() - startTime > 3000){
    //         SerialPort.println(1);
    //     }
    // }

}

int val = 0;
int loopCount = 0;
int g = 0;
void loop() {


// String msg;
int msg = -1;
if (SerialPort.available() ){
    msg = SerialPort.parseInt(); //parseInt();//('\n');
    //Serial.print(loopCount++);
    // Serial.println(msg);

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    //display.print(loopCount);
    display.println(msg);
    display.display();
    } 
//     else {
//         Serial.println(".");
//     }



//     switch (currentState)
// {
// case START:
//     //forkliftServo.write(FORKLIFTSERVO_READY_POS);
//     if(SerialPort.available()){
//     int receivedVal = SerialPort.parseInt();
//         if(receivedVal == 1) { // 1 is the signal indicating that the BP has finished driving 
//             currentState = PROCESS_STATION_4;
//         }
//     }
//     break;

// case PROCESS_STATION_4:
//     lazySusanSystem.moveToValue(NINETY_LAZYSUSAN); 
//     SerialPort.println(2);
//     //wait for bp to adjust height 
//     if(SerialPort.available()){
//         int receivedVal = SerialPort.parseInt();
//         if(receivedVal == 3) {//elevator has moved to forklift height
//             linearArmSystem.moveToValue(CLAW_FORWARD); 
//             Serial.println(4); 
//         }
//     }
//     if(SerialPort.available()){
//         int receivedVal = SerialPort.parseInt();
//         if(receivedVal == 1) {//lifted elvator forklift above table
//             currentState = PROCESS_STATION_6;
//         }
//     }
//     break;

//     default:
//         currentState = IDLE;
//     break;

//}

} //loop

void IRAM_ATTR isrUpdateLinearArmEncoder(){

    // bool A = digitalRead(linearArmEncoder.getPinA());
    // bool B = digitalRead(linearArmEncoder.getPinB());
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


// void isrupdateEncoder() {
//     bool A = digitalRead(LAZY_SUSAN_ROTARY_ENCODER_PA);
//     bool B = digitalRead(LAZY_SUSAN_ROTARY_ENCODER_PB);
//     /*	encodes 2 bit current state  */
//     int encoded = ( A << 1 ) | B;
//     // Serial.println("encoded: " + encoded);
//     // Serial.println(encoded, BIN);
//     /*	encodes the last states bits, concat the current states bits  */
//     int concat = ( lastEncoded << 2 ) | encoded;
//     // Serial.print("concat: ");
//     // Serial.println(concat, BIN);
//     //Serial.println("concat: " + String(concat));
//     /*	hard codes all the possibilities of encoded data  */
//     if (concat == 0b1101 || concat == 0b0100 || concat == 0b0010 || concat == 0b1011){
//        increments++;
//     }
//     if (concat == 0b1110 || concat == 0b0111 || concat == 0b0001 || concat == 0b1000) {
//         increments--;
//     }
//     /*	the current states bits become the next states previous bits  */
//     lastEncoded = encoded;
// }


#endif