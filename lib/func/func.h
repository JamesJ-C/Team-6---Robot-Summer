#ifndef FUNC_H
#define FUNC_H


#include <Arduino.h>


namespace funcs {
    #ifdef ESP32

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