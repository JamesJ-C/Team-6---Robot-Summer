#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#include<Arduino.h>


namespace encoder {


    class RotaryEncoder {

    private:


    const int clicksPerRotation = 20; //depending on each encoder

    PinName pinA;
    PinName pinB;


    /*  for calculating direction. This value seems to make 0 position work on startup 
    otherwise there would be an offset on increment by 2 */
    int lastEncoded = 0;//0b11; 
    volatile int increments = 10; //net clicks offset from 0
    int previousIncrement; //net clicks at the previous time stamp
    double angularVelocity = 0; 

    unsigned long lastUpdateTime = 0; //time at which the ISR was last called
    int deltaT; //ftime between the most recent ISR calls
    
    
    int maxIncrement;


    /*  Helper functions  */

    /**
     * @brief updates the speed of the encoder. Must be called after the ISR call, but before the next ISR call
     * 
     */
    void updateSpeed();


    public:

    /**
     * @brief Construct a new Rotary Encoder object
     * 
     * @param pinA pin attached to one terminal of the encoder
     * @param pinB pin attached to the other terminal of the encoder
     */
    RotaryEncoder(PinName pinA, PinName pinB);

    /**
     * @brief Get the Pin A object
     * 
     * @return PinName pin attached to the first terminal
     */
    PinName getPinA();


    /**
     * @brief Get the Pin B object
     * 
     * @return PinName pin attached to the second terminal
     */
    PinName getPinB();

    /**
     * @brief Get the number of increments offset the encoder has
     * 
     * @return int number of increments from zero position
     */
    int getIncrements();


    /**
     * @brief sets the max increment
     * 
     */

    int getMaxIncrement();

    /**
     * @brief Set the Max Increment value which corresponds to the extreme value of the encoder. 
     * This value is not enforced by the encoder object at all 
     * 
     * @param max value which corresponds to the max 
     */
    void setMaxIncrement(int max);
    
    /**
     * @brief Get the Speed measured by the encoder
     * 
     * @return int speed
     */
    double getSpeed();


    /**
     * @brief Updates the time of the most recent ISR call 
     * and the time between the 3 most recent calls
     * 
     * @param time current time
     */
    void updateTime(unsigned long time);


    void updateEncoder(bool A, bool B);

    /**
     * @brief resets the increment to 0
     * 
     */
    void resetIncrement();

    /**
     * @brief Set the Increment 
     * 
     * @param increment value to set the increment to
     */
    void setIncrement(int increment);

    };


}


#endif