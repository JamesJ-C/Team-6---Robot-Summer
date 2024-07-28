
#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>


/*  libraries we wrote  */
#include <Motor.h>
#include <RotaryEncoder.h>
#include <RobotSystems.h>
#include <robotConstants.h>
#include <claw.h>

//#ifdef ESP32
#include <ESp32Servo.h>
//#endif


void moveServo(Servo& servo);


void setup() {

    Serial.begin(115200);


}


void loop() {

}


void moveServo(Servo& servo){



} 


// void drivePID(bool direction) {

//   const int forwardMidMotorSpeed = 3300;
//   const int backwardMidMotorSpeed = 3300;

//   if (!direction) {
//     forwardError = (double) analogRead(FRONT_TAPE_SENSOR_1) - analogRead(FRONT_TAPE_SENSOR_2);


//     double FORWARD_LOOP_GAIN = 1.0;
//     double FORWARD_P_GAIN = 0.55;//1.4 goes very slowl
//     double FORWARD_I_GAIN = 0.0;
//     double FORWARD_D_GAIN = 1.36;//0.9;//0.4;//0.7;//0.9;//1.8;//1.9;//2.5;//2.0;//1.8;//0.9;//0.7

//     forward_p = FORWARD_P_GAIN * forwardError;
//     forward_d = FORWARD_D_GAIN * (forwardError - forwardLastError);
//     forward_i = FORWARD_I_GAIN * forwardError + forward_i; //const * error + previous int value
//     if (forward_i > max_I) {forward_i = max_I;}
//     if (forward_i < -max_I) {forward_i = -max_I;}

//     forward_g = FORWARD_LOOP_GAIN * ( forward_p + forward_i + forward_d ); 
//     forwardLastError = forwardError; 

//     //SEND MOTOR VALS

//     MotorL.forward( (forwardMidMotorSpeed - 1 * forward_g) );
//     MotorR.forward(  1 / 1.3 * ( ( forwardMidMotorSpeed + 1 * forward_g) ) );

//     MotorR.forward(3300);
//     MotorL.forward(3300);
//     /*  SerialPort & Serial Monitor prints  */
//     {

//     int mL = forwardMidMotorSpeed - forward_g;
//     int mR = forwardMidMotorSpeed + forward_g;
//     // SerialPort.println("g: " + String( g) );
//     // SerialPort.println("m1: " + String( midMotorSpeed - g) );
//     // SerialPort.println("m2: " + String( midMotorSpeed + g) );

//     // Serial.println( "tape 1: " + String( analogRead(FRONT_TAPE_SENSOR_1) ));
//     // Serial.println( "tape 2: " + String( analogRead(FRONT_TAPE_SENSOR_2 ) ));

//     Serial.println( "error: " + String( forwardError ));
//     Serial.println( "p: " + String( forward_p ));

//     //SerialPort.println( "tape 1: " + String( analogRead(FRONT_TAPE_SENSOR_1) ));
//     //SerialPort.println( "tape 2: " + String( analogRead(FRONT_TAPE_SENSOR_2 ) ));

//     Serial.print("g: ");
//     Serial.println(forward_g);

//     Serial.print("mL: ");
//     Serial.println(mL);

//     Serial.print("mR: ");
//     Serial.println(mR);

//     }
//   }
//   else {
//     backwardError = (double) analogRead(BACK_TAPE_SENSOR_1) - analogRead(BACK_TAPE_SENSOR_2);


//     double BACKWARD_LOOP_GAIN = 1.0;
//     double BACKWARD_P_GAIN = 0.55;//1.4 goes very slowl
//     double BACKWARD_I_GAIN = 0.0;
//     double BACKWARD_D_GAIN = 1.36;//0.9;//0.4;//0.7;//0.9;//1.8;//1.9;//2.5;//2.0;//1.8;//0.9;//0.7

//     backward_p = BACKWARD_P_GAIN * backwardError;
//     backward_d = BACKWARD_D_GAIN * (backwardError - backwardLastError);
//     backward_i = BACKWARD_I_GAIN * backwardError + backward_i; //const * error + previous int value
//     if (backward_i > max_I) {backward_i = max_I;}
//     if (backward_i < -max_I) {backward_i = -max_I;}

//     backward_g = BACKWARD_LOOP_GAIN * ( backward_p + backward_i + backward_d ); 
//     backwardLastError = backwardError; 

//     //SEND MOTOR VALS

//     MotorL.backward( (backwardMidMotorSpeed - 1 * backward_g) );
//     MotorR.backward(  1 / 1.2 * ( ( backwardMidMotorSpeed + 1 * backward_g) ) );


//     /*  SerialPort & Serial Monitor prints  */
//     {

//     int mL = backwardMidMotorSpeed - backward_g;
//     int mR = backwardMidMotorSpeed + backward_g;
//     // SerialPort.println("g: " + String( g) );
//     // SerialPort.println("m1: " + String( midMotorSpeed - g) );
//     // SerialPort.println("m2: " + String( midMotorSpeed + g) );

//     Serial.println( "tape 1: " + String( analogRead(FRONT_TAPE_SENSOR_1) ));
//     Serial.println( "tape 2: " + String( analogRead(FRONT_TAPE_SENSOR_2 ) ));

//     Serial.println( "error: " + String( backwardError ));
//     Serial.println( "p: " + String( backward_p ));

//     SerialPort.println( "tape 1: " + String( analogRead(FRONT_TAPE_SENSOR_1) ));
//     SerialPort.println( "tape 2: " + String( analogRead(FRONT_TAPE_SENSOR_2 ) ));

//     Serial.print("g: ");
//     Serial.println(backward_g);

//     Serial.print("mL: ");
//     Serial.println(mL);

//     Serial.print("mR: ");
//     Serial.println(mR);

//     }
 
//   }

// }


