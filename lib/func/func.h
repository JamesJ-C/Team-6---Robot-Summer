#ifndef James_FUNC_H
#define JAMES_FUNC_H


#include <Arduino.h>


namespace funcs {
    
    #ifdef ESP32

    static void printStuff(){

        Serial.println("esp board: ");


    }

    #endif

    #ifdef BLUEPILL_F103C8

    static void printStuff(){

        Serial.println("esp board: ");


    }
    #endif

    #ifndef ESP32 

    static void printStuff(){
        Serial.println("not esp.");
    }


    #endif

}

#endif