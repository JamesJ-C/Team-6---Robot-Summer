#include <Arduino.h>


/**
 * @brief 
 * 
 */
class RotaryEncoder {

  private:


  const int clicksPerRotation = 20; //depending on each encoder

  PinName pinA;
  PinName pinB;


  /*  for calculating direction. This value seems to make 0 position work on startup 
  otherwise there would be an offset on increment by 2 */
  int lastEncoded = 0b11; 
  int increments = 0; //net clicks offset from 0
  int previousIncrement; //net clicks at the previous time stamp
  double angularVelocity = 0; 

  unsigned long lastUpdateTime = 0; //time at which the ISR was last called
  int deltaT; //ftime between the most recent ISR calls
  
  
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

  
  /**
   * @brief updates the speed of the encoder. Must be called after the ISR call, but before the next ISR call
   * 
   */
  void updateSpeed();

  /**
   * @brief old function not needed. Still would like to keep for the time being
   * 
   * @param Aa 
   * @param Bb 
   */
  void updateEncoder(bool Aa, bool Bb);

  
  /**
   * @brief Updates the encoder object. Should be called from an ISR
   * 
   */
  void updateEncoder();

};
