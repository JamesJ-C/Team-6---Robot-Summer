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
  RotaryEncoder(PinName pinA, PinName pinB);// : pinA(pinA), pinB(pinB);

  /**
   * @brief Get the Pin A object
   * 
   * @return PinName pin attached to the first terminal
   */
  PinName getPinA(){
    return pinA;
  }
  /**
   * @brief Get the Pin B object
   * 
   * @return PinName pin attached to the second terminal
   */
  PinName getPinB(){
    return pinB;
  }
  /**
   * @brief Get the number of increments offset the encoder has
   * 
   * @return int number of increments from zero position
   */
  int getIncrements(){
    return increments;
  }
  /**
   * @brief Get the Speed measured by the encoder
   * 
   * @return int speed
   */
  double getSpeed(){
    this->updateSpeed();
    return angularVelocity;
  }

  
  /**
   * @brief Updates the time of the most recent ISR call 
   * and the time between the 3 most recent calls
   * 
   * @param time current time
   */
  void updateTime(unsigned long time){

    deltaT = time - this->lastUpdateTime;
    this->lastUpdateTime = time;
  }

  
  /**
   * @brief updates the speed of the encoder. Must be called after the ISR call, but before the next ISR call
   * 
   */
  void updateSpeed(){


    //THIS FUNCTION DOES NOT WORK
    angularVelocity =  ( (double) (this->increments - this->previousIncrement) / deltaT ) / (double) clicksPerRotation;

  }

  /**
   * @brief old function not needed. Still would like to keep for the time being
   * 
   * @param Aa 
   * @param Bb 
   */
  void updateEncoder(bool Aa, bool Bb){

    // bool A = digitalRead(ROTARY_A);
    // bool B = digitalRead(ROTARY_B);

    // /*	encodes 2 bit current state  */
    // int encoded = ( A << 1 ) | B;
    // /*	encodes the last states bits, concat the current states bits  */
    // int concat = ( lastEncoded << 2 ) | encoded;

    // /*	hard codes all the possibilities of encoded data  */
    // if (concat == 0b1101 || concat == 0b0100 || concat == 0b0010 || concat == 0b1011){
    //   this->increments++;
    // }
    // if (concat == 0b1110 || concat == 0b0111 || concat == 0b0001 || concat == 0b1000) {
    //   this->increments--;
    // }

    // /*	the current states bits become the next states previous bits  */
    // this->lastEncoded = encoded;


  }

  
  /**
   * @brief Updates the encoder object. Should be called from an ISR
   * 
   */
  void updateEncoder(){

    bool A = digitalRead(pinA);
    bool B = digitalRead(pinB);

    /*	encodes 2 bit current state  */
    int encoded = ( A << 1 ) | B;
    /*	encodes the last states bits, concat the current states bits  */
    int concat = ( lastEncoded << 2 ) | encoded;

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

};
