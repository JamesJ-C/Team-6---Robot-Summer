#include <Arduino.h>
#include "RotaryEncoder.h"



namespace encoder {


    /**
     * @brief Construct a new Rotary Encoder object
     * 
     * @param pinA pin attached to one terminal of the encoder
     * @param pinB pin attached to the other terminal of the encoder
     */
    RotaryEncoder::RotaryEncoder(PinName pinA, PinName pinB) : pinA(pinA), pinB(pinB) {}

    /**
     * @brief Get the Pin A object
     * 
     * @return PinName pin attached to the first terminal
     */
    PinName RotaryEncoder::getPinA(){
        return pinA;
    }
    /**
     * @brief Get the Pin B object
     * 
     * @return PinName pin attached to the second terminal
     */
    PinName RotaryEncoder::getPinB(){
        return pinB;
    }
    /**
     * @brief Get the number of increments offset the encoder has
     * 
     * @return int number of increments from zero position
     */
    int RotaryEncoder::getIncrements(){
        return increments;
    }


    void RotaryEncoder::setMaxIncrement(int max){
        this->maxIncrement = max;
    }

    int RotaryEncoder::getMaxIncrement(){
        return this->maxIncrement;
    }
    
    /**
     * @brief Get the Speed measured by the encoder
     * 
     * @return int speed
     */
    double RotaryEncoder::getSpeed(){
        this->updateSpeed();
        return angularVelocity;
    }

    
    /**
     * @brief Updates the time of the most recent ISR call 
     * and the time between the 3 most recent calls
     * 
     * @param time current time
     */
    void RotaryEncoder::updateTime(unsigned long time){

        deltaT = time - this->lastUpdateTime;
        this->lastUpdateTime = time;
    }

    
    /**
     * @brief updates the speed of the encoder. Must be called after the ISR call, but before the next ISR call
     * 
     */
    void RotaryEncoder::updateSpeed(){


        //THIS FUNCTION DOES NOT WORK
        angularVelocity =  ( (double) (this->increments - this->previousIncrement) / deltaT ) / (double) clicksPerRotation;

    }

    

    
    /**
     * @brief Updates the encoder object. Should be called from an ISR
     * 
     */
    void RotaryEncoder::updateEncoder(){

        bool A = digitalRead(this->pinA);
        bool B = digitalRead(this->pinB);


        /*	encodes 2 bit current state  */
        int encoded = ( A << 1 ) | B;
        /*	encodes the last states bits, concat the current states bits  */
        int concat = ( lastEncoded << 2 ) | encoded;

        Serial.println("concat: " + String(concat, BIN));

        /*	hard codes all the possibilities of encoded data  */
        if (concat == 0b1101 || concat == 0b0100 || concat == 0b0010 || concat == 0b1011){
        this->increments++;
        }
        if (concat == 0b1110 || concat == 0b0111 || concat == 0b0001 || concat == 0b1000) {
        this->increments--;
        }

        /*	the current states bits become the next states previous bits  */
        this->lastEncoded = encoded;

    }

     /**
     * @brief old function not needed. Still would like to keep for the time being
     * 
     * @param Aa 
     * @param Bb 
     */
    void RotaryEncoder::updateEncoder(bool A, bool B){

        // bool A = digitalRead(ROTARY_A);
        // bool B = digitalRead(ROTARY_B);

        /*	encodes 2 bit current state  */
        int encoded = ( A << 1 ) | B;
        /*	encodes the last states bits, concat the current states bits  */
        int concat = ( lastEncoded << 2 ) | encoded;


        Serial.println("concat: " + String(concat));

        /*	hard codes all the possibilities of encoded data  */
        if (concat == 0b1101 || concat == 0b0100 || concat == 0b0010 || concat == 0b1011){
          this->increments++;
        }
        if (concat == 0b1110 || concat == 0b0111 || concat == 0b0001 || concat == 0b1000) {
          this->increments--;
        }

        /*	the current states bits become the next states previous bits  */
        this->lastEncoded = encoded;


    }

    /**
     * @brief resets the increment to 0
     * 
     */
    void RotaryEncoder::resetIncrement(){

        this->increments = 0;

    }

    void RotaryEncoder::setIncrement(int increment) {
        this->increments = increment;
    }







}