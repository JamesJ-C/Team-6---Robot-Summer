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


void setup() {
  display_handler.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 
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

  // Set up PA1 as input
  pinMode(REFLECTANCE, INPUT);
  // Set up interrupt function
  //attachInterrupt(digitalPinToInterrupt(REFLECTANCE), handle_state_change, CHANGE);
  sample = 0;
}

void loop() {

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

  }

  convolution(measuredArray, measuredArraySize, sineArray, sineArraySize, convolvedArray, convolvedArraySize);

  if (convolvedArray[0] == 0){
    return -9999999;
  }

  return findMax(convolvedArray, convolvedArraySize);


}


int findMax(int* array, int arraySize) {

  int max = array[0];
  for (int i = 1; i <arraySize; i++) {

    if (array[i] > max){
      max = array[i];
    }

  }

  return max;

}


int findMin(int* array, int arraySize) {

  int min = array[0];
  for (int i = 1; i <arraySize; i++) {

    if (array[i] < min){
      min = array[i];
    }

  }

  return min;

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