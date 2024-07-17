
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <vector>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define REFLECTANCE PA_0 // Input pin
#define IR_SENSOR1 PA_0
#define IR_SENSOR2 PA_1
#define THRESHOLD 100 // Black line detection threshold
#define NUM_SAMPLES 200

int loopCount = 0;
double minTot = 9999999;
double maxTot = 0;
double avg = 0;


/*  Function decs  */
double crossCorrelation (PinName analogPin);
std:: vector<double> bothCrossCorrelation (PinName analogPin1, PinName analogPin2);



/**
 * @brief Object of an IR sensor
 * 
 */
class IrSensor {


  private:
  PinName sensorPin; //pin sensor is attached to
  double lastCorrelationVal = -1; //result of the most recent correlation

  public:

  /**
   * @brief Construct a new Ir Sensor object
   * 
   * @param pin the pin which the ir detector is connected to
   */
  IrSensor(PinName pin) : sensorPin(pin) {}

  /**
   * @brief Get the most recent result of this->crossCorrelation().
   * 
   * @return double most recent result of crossCorrelation.
   * returns -1 if crossCorrelation() has not been called yet
   */
  double getLastCorrelationVal(){

    return this->lastCorrelationVal;

  }


  /**
   * @brief measures a signal and cross correlates it with 
   * a 1kHz sine wave. 
   * 
   * @return 'amount' of 1kHz IR signal detected
   */
  double crossCorrelation() {

    std::vector<double> IRsignal;
    int numSamples = 0;
    unsigned long finishTime = 0;
    unsigned long startTime = millis(); 

    /*  Read Values from sensor  */
    while (millis() - startTime < 10){
      IRsignal.push_back(analogRead(this->sensorPin));
      numSamples++;
      finishTime = millis();
    }

    double oneK[2* numSamples] = {0}; //computed sine wave
    double oneKCorr[numSamples] = {0}; //array for convolved result

    /*  calculate period of sine wave  */
    int dt = ( finishTime - startTime );
    double oneKT = (double) numSamples / ( (double) dt );

    /*  Create sine wave  */
    for(int i = 0; i < 2 * numSamples;  i++) {
      oneK[i] = sin(i * TWO_PI / oneKT);
    }

    /*  Convolve measured IR signal with created sin wave  */
    for (int k = 0; k < numSamples; k++){
      oneKCorr[k] = 0;
      for (int i = 0; i < numSamples; i++){      
        oneKCorr[k] += IRsignal.at(i) * oneK[k+i];
      }
    }

    /*  Find max in array  */
    double max = oneKCorr[0];
    for (int i=0; i< numSamples; i++) {

      if (oneKCorr[i]>max){
        max = oneKCorr[i];
      }
    }

    this->lastCorrelationVal = max;
    return max;

  }


};



void setup() {
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 

  Serial.begin(9600);

  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display_handler.display();
  delay(2000);

  // Displays "Hello world!" on the screen
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("Setup");
  display_handler.display();

  Serial.println("Setup");

  // Set up PA1 as input
  pinMode(REFLECTANCE, INPUT);


}


void loop() {

  //Serial.println("x correlation: " + String( crossCorrelation( REFLECTANCE ) ));
  std::vector result = bothCrossCorrelation(IR_SENSOR1, IR_SENSOR2);

  Serial.println("result 1: " + String(result.at(0)) + "\n result 2: " + String(result.at(1)));

  delay(400);

}



int PID_IR_Beacon_Control(){
  //do PID




}


double crossCorrelation (PinName analogPin){

  std::vector<double> IRsignal;

  int numSamples = 0;
  unsigned long finishTime = 0;
  unsigned long startTime = millis();

  while (millis() - startTime < 10){

    IRsignal.push_back(analogRead(analogPin));
    numSamples++;
    finishTime = millis();
  }


  double oneK[2* numSamples] = {0};
  double oneKCorr[numSamples] = {0};

  int dt = ( finishTime - startTime );
  double oneKT = (double) numSamples / ( (double) dt );

  for(int i = 0; i < 2 * numSamples;  i++) {
  
    oneK[i] = sin(i * TWO_PI / oneKT);
  
  }

  for (int k = 0; k < numSamples; k++){

    oneKCorr[k] = 0;

    for (int i = 0; i < numSamples; i++){      
      oneKCorr[k] += IRsignal.at(i) * oneK[k+i];
    }

  }

  double max = oneKCorr[0];

  for (int i=0; i< numSamples; i++) {

    if (oneKCorr[i]>max){
      max = oneKCorr[i];
    }
  }


  if (max < minTot){
    minTot = max;
  }
  if (max > maxTot){
    maxTot = max;
  }

  avg = ( (loopCount - 1) * avg + max ) / loopCount;

    return max;

}



std:: vector<double> bothCrossCorrelation (PinName analogPin1, PinName analogPin2){

  std::vector<double> IRsignal1;
  std::vector<double> IRsignal2;

  int numSamples = 0;
  unsigned long finishTime = 0;
  unsigned long startTime = millis();

  while (millis() - startTime < 10){

    IRsignal1.push_back(analogRead(analogPin1));
    IRsignal2.push_back(analogRead(analogPin2));

    numSamples++;
    finishTime = millis();
  }


  double oneK[2* numSamples] = {0};
  double oneKCorr1[numSamples] = {0};
  double oneKCorr2[numSamples] = {0};


  int dt = ( finishTime - startTime );
  double oneKT = (double) numSamples / ( (double) dt );

  for(int i = 0; i < 2 * numSamples;  i++) {
  
    oneK[i] = sin(i * TWO_PI / oneKT);
  
  }

  for (int k = 0; k < numSamples; k++){

    oneKCorr1[k] = 0;

    for (int i = 0; i < numSamples; i++){      
      oneKCorr1[k] += IRsignal1.at(i) * oneK[k+i];
      oneKCorr2[k] += IRsignal2.at(i) * oneK[k+i];
    }

  }

  double max1 = oneKCorr1[0];
  double max2 = oneKCorr2[0];

  for (int i=0; i< numSamples; i++) {

    if (oneKCorr1[i]>max1){
      max1 = oneKCorr1[i];
    }
    if (oneKCorr2[i]>max2){
      max2 = oneKCorr2[i];
    }
  }

  std::vector<double> result;

  result.push_back(max1);
  result.push_back(max2);

  return result;
    //return max;

}



