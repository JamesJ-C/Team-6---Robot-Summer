#include <Wire.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define REFLECTANCE PA0 // Input pin
#define THRESHOLD 100 // Black line detection threshold
#define NUM_SAMPLES 200
int sampleWave[NUM_SAMPLES];
int sample;
unsigned long currentTime;
unsigned long startTime;
void handle_state_change();

int loopNum=0;


/*  Function Decs */

void convolution (int* measuredArray, int measuredArraySize, int* sineArray, 
                    int sineArraySize, int* convolvedArray, int convolvedArraySize);

double irAmplitude (int* measuredArray, int measuredArraySize, int timeStep);

int findMax(int* array, int arraySize);
int findMin(int* array, int arraySize);
double crossCorelate(double* measuredWave, int measuredWaveSize, int deltaT, int numSamples);

void setup() {
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 

  Serial.begin(115200);

  // Displays Adafruit logo by default. call clearDisplay immediately if you don't want this.
  display_handler.display();
  delay(2000);

  // Displays "Hello world!" on the screen
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("Setup");
    //display_handler.println(sin(PI/2));
  display_handler.display();

  // Set up PA1 as input
  pinMode(REFLECTANCE, INPUT);
  // Set up interrupt function
  //attachInterrupt(digitalPinToInterrupt(REFLECTANCE), handle_state_change, CHANGE);
  sample = 0;
}

void loop() {

  int numSamples = 1000;
  double measuredWave[3*numSamples];

  int time0 = millis(); 
  for(int i=0; i<numSamples; i++){

    measuredWave[i] = analogRead(REFLECTANCE);

    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
    display_handler.println("i: ");
    display_handler.println(i);
    display_handler.display();

  }

  int timeF = millis();

  int deltaT = numSamples / (timeF - time0);

  double CC = crossCorelate(measuredWave, 3*numSamples, deltaT, numSamples);


  
  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.println("cross correlation: ");
  display_handler.println(CC);
  display_handler.display();

/*
loopNum++;
sample = -1;

  int startTime = millis();
  int sampleNum = 0;
  while (sampleNum < NUM_SAMPLES){

    sampleWave[sampleNum] = analogRead(PA0);
    sampleNum++;
  }
  int finishTime = millis();

  int sampleTime = finishTime - startTime;

  int ConvVal = irAmplitude(sampleWave, NUM_SAMPLES, sampleTime/NUM_SAMPLES);

  for (int i = 0; i < NUM_SAMPLES; i++) {
    sampleWave[i] = 0;
  }

  display_handler.clearDisplay();
  display_handler.setCursor(0,0);
  display_handler.print("conv. val: ");
  display_handler.println(ConvVal);

  display_handler.print("loopNum: ");
  display_handler.println(loopNum);

  display_handler.print("Max in sample: ");
  display_handler.println(findMax(sampleWave, NUM_SAMPLES));

  display_handler.print("Min in sample: ");
  display_handler.println(findMin(sampleWave, NUM_SAMPLES));

  display_handler.display();


/**
  if (sample == NUM_SAMPLES) {
    currentTime = millis();
    int timeStep = (currentTime - startTime) / NUM_SAMPLES;
    
    loopNum++;

    double ConvVal = irAmplitude(sampleWave, NUM_SAMPLES, timeStep);

    display_handler.clearDisplay();
    display_handler.setCursor(0,0);
    display_handler.print("conv. val: ");
    display_handler.println(ConvVal);

    display_handler.print("loopNum: ");
    display_handler.println(loopNum);

    display_handler.print("Max in sample: ");
    display_handler.println(findMax(sampleWave, NUM_SAMPLES));

    display_handler.print("Min in sample: ");
    display_handler.println(findMin(sampleWave, NUM_SAMPLES));


    display_handler.display();
    
    // call james' function
    // display intensity of 1 kHz and 10 kHz
    // restart sample sequence
    for (int i = 0; i < NUM_SAMPLES; i++) {
      sampleWave[i] = 0;
    }
    sample = 0;
  }

  if (sample == 0) {
    startTime = millis();
  }

 // currentTime = millis();
  // sampleWave[sample] = analogRead(PA0);
  // sample++;


*/
}


/**
 * @brief Returns the max double in an array
 * 
 * @param array 
 * @param arraySize 
 * @return double 
 */
double findMax(double* array, int arraySize) {

  double max = array[0];
  for (int i = 1; i <arraySize; i++) {

    if (array[i] > max){
      max = array[i];
    }

  }

  return max;

}

/**
 * @brief returns the min double in an array
 * 
 * @param array 
 * @param arraySize 
 * @return int 
 */
double findMin(double* array, int arraySize) {

  double min = array[0];
  for (int i = 1; i <arraySize; i++) {

    if (array[i] < min){
      min = array[i];
    }

  }

  return min;

}


/**
 * @brief Creates a sin wave
 * 
 * @param sineWave 
 * @param period 
 * @param sampleSize the size of the sineWave array sent
 */
void sinWave(double* sineWave, int period, int arraySize){

  for (int i=0; i<=arraySize; i++){
    
    sineWave[i] = sin( (double) ( i * 2*PI / (double) period ) );

  }

}

/**
 * @brief 
 * 
 * @param measuredWave wave measured
 * @param measuredWaveSize size of the measured wave
 * @param numSamples 
 * @return double 
 */
double crossCorelate(double* measuredWave, int measuredWaveSize, int deltaT, int numSamples){


  int correlate[numSamples];
  double sinWaveArray[measuredWaveSize*2];

  /*  Create sin wave  */
  sinWave(sinWaveArray, deltaT, measuredWaveSize*2);

  /*  Do correlation  */
  for (int k=0; k < numSamples; k++){

    correlate[k]=0;

    for (int i=0; i < numSamples; i++){

      correlate[k] += measuredWave[k]*sinWaveArray[i+k];
    }
  }  

  //double max = findMax(correlate, measuredWaveSize);
  //double min = findMin(correlate, measuredWaveSize);



  double max = correlate[0];
  for (int i = 1; i <numSamples; i++) {

    if (correlate[i] > max){
      max = correlate[i];
    }

  }

  return max;
}




void handle_state_change() {
  display_handler.clearDisplay();
  display_handler.setCursor(0,0);
  display_handler.println("Reflectance value2:\n");
  //display_handler.println(analogRead(REFLECTANCE));
  display_handler.display();
}


/**
 * 
 * @param measuredArray: the array of values measured from the IR sensor
 * @param measuredArraySize: the size of the measured array
 * 
 * @returns 'amount' of 1 kHz frequency in the measured array wave
 * 
 * 
*/

double irAmplitude (int* measuredArray, int measuredArraySize, int timeStep){


  /** Create Sine Wave  **/
  int convolvedArraySize = measuredArraySize*2;
  const int frequency = 1000;
  int sineArraySize = convolvedArraySize;
  int sineArray[sineArraySize];


  int convolvedArray[convolvedArraySize];
  convolvedArray[0] = 0;

  /*  Create Sine wave  */
  for (int x = 0; x < sineArraySize; x++) {

    sineArray[x] = sin(2*PI*frequency*x*timeStep);
    Serial.println(sineArray[x]);

  }

  convolution(measuredArray, measuredArraySize, sineArray, sineArraySize, convolvedArray, convolvedArraySize);

  if (convolvedArray[0] == 0){
    return -9999999;
  }

  return findMax(convolvedArray, convolvedArraySize);


}


void convolution (int* measuredArray, int measuredArraySize, int* sineArray, 
                    int sineArraySize, int* convolvedArray, int convolvedArraySize) {


//200 = measured array size
//  int measuredArray[200];
  //int sinArray[400];
  //int convolvedArray[200];

  for (int offset = 0; offset <= measuredArraySize; offset++){ // offset of 0 is 1 overlap, offset of

    for (int i = 0; i <= offset; i++){ // < or <= offset???

      //offset of 0 corresponds to 1 overlapping value, namely, the first value of the measured, 
      //and last value of the created 
      convolvedArray[offset] = measuredArray[i] *sineArray[sineArraySize-offset+i];

    }

  }

  //return convolvedArray;


}