
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <vector>

#include "RotaryEncoder.h"
#include "Motor.h"
#include "robotConstants.h"


Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


int loopCount = 0;
double minTot = 9999999;
double maxTot = 0;
double avg = 0;


/*  Function decs  */
double crossCorrelation (PinName analogPin);
std:: vector<double> bothCrossCorrelation (PinName analogPin1, PinName analogPin2);
double PID_IR_Beacon_Control(double irVal1, double irVal2);


/*  Constants for IR  */

const int IR_ERROR_THRESHOLD = 50;

double LOOP_GAIN = 1/50.0;
int P_GAIN = 30;
int I_GAIN = 0;
int D_GAIN = 0;

/*  values used in IR PID  */
int error = 0;
int lastError = 0;
int max_I = 140;



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


}


void loop() {

  //Serial.println("x correlation: " + String( crossCorrelation( REFLECTANCE ) ));
  std::vector result = bothCrossCorrelation(IR_SENSOR1, IR_SENSOR2);

  Serial.println("result 1: " + String(result.at(0)) + "\n result 2: " + String(result.at(1)));

  int g = PID_IR_Beacon_Control(result.at(0), result.at(1));

  Serial.println("transfer function: " + String(g));

  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.print("g: ");
  display_handler.println( g );
  display_handler.display();


  delay(600);

}

/*  Tuning this is weird, bc as the sensors get farther from the signal, 
the difference in readings approach 0. This will probably be different for sensors which are further away from the beacon,
and angled better  */
double PID_IR_Beacon_Control(double irVal1, double irVal2){
  //do PID

  int g;
  int p,d,i;

  error = irVal1 - irVal2;


  if ( abs(error) < IR_ERROR_THRESHOLD){
    error = 0;
  }



  p = P_GAIN * error;
  d = D_GAIN * (error - lastError);
  i = I_GAIN * error + i; //const * error + previous int value
  if (i > max_I) {i = max_I;}
  if (i < -max_I) {i = -max_I;}


  g = LOOP_GAIN * ( p + i + d );

  lastError = error;

  return g;


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



