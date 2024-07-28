#ifndef ROBOT_CLAW_H
#define ROBOT_CLAW_H

#include <Arduino.h>
#include <ESP32Servo.h>

namespace clawActuation {


class clawidk {

    clawidk (Servo& clawServo, Servo& forkliftServo);


};


}

#endif