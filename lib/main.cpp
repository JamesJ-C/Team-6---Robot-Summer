#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>

#include <HardwareSerial.h>

#include <RotaryEncoder.h>
#include <Motor.h>
#include <robotConstants.h>

/*  BP pin defs for UART */

#define RX PB11
#define TX PB10

HardwareSerial SerialPort(USART3);
String msg;


/*  Function Declerations  */
void ISRUpdateLinearArmEncoder();
void ISRUpdateElevatorEncoder();
void localize();


/*  PID Control Values  */

int setVal = 32;
int measuredVal;

/*  arm PID  */
double armError = 0.0;
double lastArmError = 0.0;
double MAX_I = 140;
double p_arm_Val, d_arm_Val, i_arm_Val;
double g_arm_Val ;


/*  Elevator PID  */
double plateError = 0.0;
double lastPlateError = 0.0;
double MAX_I = 140;
double p_elevator_Val, d_elevator_Val, i_elevator_Val;
double g_elevator_Val ;



/*  Object declerations  */

encoder::RotaryEncoder armEncoder(PB_8, PB_9);
movement::Motor armMotor(ARM_MOTOR_P1, ARM_MOTOR_P2, &armEncoder);

encoder::RotaryEncoder elevatorEncoder( ELEVATOR_ROTARY_ENCODER_A, ELEVATOR_ROTARY_ENCODER_B);
movement::Motor MotorElevator(MOTOR_ELEVATOR_P1, MOTOR_ELEVATOR_P2, &elevatorEncoder);


void setup() {

  /*  Setup Serials  */
  {
    /*  Setup Serial Monitor  */
    Serial.begin(115200);
    Serial.println("Setup..." + String(BOARD_NAME));

    /*  Setup UART port  */
    SerialPort.begin(115200);

  }

  /*  PinModes  */
  {
    /*  Motor Pins  */
    pinMode(armMotor.getPinA(), OUTPUT);
    pinMode(armMotor.getPinB(), OUTPUT);

    pinMode(MotorElevator.getPinA(), OUTPUT);
    pinMode(MotorElevator.getPinB(), OUTPUT);

    /*  Encoders  */
    pinMode(armEncoder.getPinA(), INPUT);
    pinMode(armEncoder.getPinB(), INPUT);

    pinMode(elevatorEncoder.getPinA(), INPUT);
	pinMode(elevatorEncoder.getPinB(), INPUT);    

  }

  /*  Attach Interrupts  */
  {
    /*  Arm encoders  */
    attachInterrupt(digitalPinToInterrupt(armEncoder.getPinA()), ISRUpdateLinearArmEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(armEncoder.getPinB()), ISRUpdateLinearArmEncoder, CHANGE);
  
    attachInterrupt(digitalPinToInterrupt( elevatorEncoder.getPinA() ), ISRUpdateElevatorEncoder, CHANGE);
	attachInterrupt(digitalPinToInterrupt( elevatorEncoder.getPinB() ), ISRUpdateElevatorEncoder, CHANGE);
  
  }

  /*  Perform Setup Actions  */
  {

    /*  Arm setup  */
    armMotor.off();
    delay(500);
    armMotor.forward(3000);
    delay(100);
    armMotor.off();
    delay(100);

    /*  Elevator setup  */
    MotorElevator.off();
    delay(500);
    MotorElevator.forward(3000);
    delay(100);
    MotorElevator.off();
    delay(100);

    /*  perform motor sweep to initialize motion  */
    localize();

  }

}


void loop() {

}


/**
 * @brief function attached to RotaryA and RotaryB to update encoder values
 * 
 */
void ISRUpdateLinearArmEncoder(){

  bool A = digitalRead( armEncoder.getPinA() );
  bool B = digitalRead( armEncoder.getPinB() );

  armEncoder.updateEncoder(A, B);
  armEncoder.updateTime( millis() );

}

/**
 * @brief function attached to RotaryA and RotaryB to update encoder values
 * 
 */
void ISRUpdateElevatorEncoder(){

  bool A = digitalRead( elevatorEncoder.getPinA() );
  bool B = digitalRead( elevatorEncoder.getPinB() );

  elevatorEncoder.updateEncoder(A, B);
  elevatorEncoder.updateTime( millis() );

}


/**
 * @brief performs motor sweep to localize range of elevator motion 
 * updates encoder range values and sends the motor to the center once completed
 * 
 */
void localize() {

    int bottom;
    int top;
    int center;

    // turn motor until elevator reaches bottom limit
    while (!digitalRead(ARM_LOWER_LIMIT_SWITCH)) {
        armMotor.backward(2000);
    }

    // initialize bottom of elevator movement
    armMotor.off();
    armMotor.encoder->resetIncrement();
    bottom = armMotor.encoder->getIncrements();

    // turn motor in opposite direction until top limit reached
    while (!digitalRead(ARM_UPPER_LIMIT_SWITCH)) {
        armMotor.forward(2000);
    }

    // initialize top of elevator movement
    armMotor.off();
    top = armMotor.encoder->getIncrements();
    armMotor.encoder->setMaxIncrement(top);

    // turn motor and reach middle of motion
    center = top / 2;
    while (armMotor.encoder->getIncrements() != center) {
        armMotor.backward(3000);
    }
    armMotor.stop();

}