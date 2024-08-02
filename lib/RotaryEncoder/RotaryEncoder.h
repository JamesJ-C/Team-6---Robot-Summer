#ifndef ROTARY_ENCODER_H 
#define ROTARY_ENCODER_H 

#include<Arduino.h>


#ifdef ESP32
#include <bpConstants.h>
#endif

#ifndef ESP32
#include <espConstants.h>
#endif

namespace encoder {


    class RotaryEncoder {

    private:


    const int clicksPerRotation = 20; //depending on each encoder

    uint8_t pinA;
    uint8_t pinB;


    /*  for calculating direction. This value seems to make 0 position work on startup 
    otherwise there would be an offset on increment by 2 */
    int lastEncoded = 0b11; 
    volatile int increments = 0; //net clicks offset from 0
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
    RotaryEncoder(uint8_t pinA, uint8_t pinB);

    /**
     * @brief Get the Pin A object
     * 
     * @return uint8_t pin attached to the first terminal
     */
    uint8_t getPinA();


    /**
     * @brief Get the Pin B object
     * 
     * @return uint8_t pin attached to the second terminal
     */
    uint8_t getPinB();

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

    void updateEncoder();

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