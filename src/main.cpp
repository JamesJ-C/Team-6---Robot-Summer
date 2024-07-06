
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <vector>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible
Adafruit_SSD1306 display_handler(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define REFLECTANCE PA0 // Input pin
#define IRSENSOR PA0
#define THRESHOLD 100 // Black line detection threshold
#define NUM_SAMPLES 200

int loopCount = 0;
double minTot = 9999999;
double maxTot = 0;
double avg = 0;

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

  pinMode(PB11, OUTPUT);
  digitalWrite(PB11, LOW);
  delay(100);

  digitalWrite(PB11, HIGH);
  delay(100);
  // Set up interrupt function
  //attachInterrupt(digitalPinToInterrupt(REFLECTANCE), handle_state_change, CHANGE);

}

void loop() {

loopCount++;
//double measuredWave[3*numSamples];

  std::vector<double> IRsignal;

  int numSamples = 0;
  unsigned long finishTime = 0;
  unsigned long startTime = millis(); 

    // display_handler.clearDisplay();
    // display_handler.setTextSize(1);
    // display_handler.setTextColor(SSD1306_WHITE);
    // display_handler.setCursor(0,0);
    // display_handler.print("before while ");
    // //display_handler.print();
    // display_handler.println();
    // display_handler.display();
    // delay(500);



/*
  Serial.println("outside loop");

  display_handler.clearDisplay();
  display_handler.setTextSize(1);
  display_handler.setTextColor(SSD1306_WHITE);
  display_handler.setCursor(0,0);
  display_handler.print("startTime: ");
  display_handler.print(startTime);
  display_handler.println();
  display_handler.display();  


  while(millis() - startTime < 10) {

    digitalWrite(PB11, LOW);




  } */

  //digitalWrite(PB11, HIGH);

 // /*
  while (millis() - startTime < 10){
    //digitalWrite(PB11, LOW);
    IRsignal.push_back(analogRead(IRSENSOR));
    numSamples++;
    finishTime = millis();

    // display_handler.clearDisplay();
    // display_handler.setTextSize(1);
    // display_handler.setTextColor(SSD1306_WHITE);
    // display_handler.setCursor(0,0);
    // display_handler.print("numSamples: ");
    // display_handler.println(numSamples);

    // // display_handler.print("loop count: ");
    // // display_handler.println(loopCount);
    // display_handler.display();

  }

  //   display_handler.clearDisplay();
  //   display_handler.setTextSize(1);
  //   display_handler.setTextColor(SSD1306_WHITE);
  //   display_handler.setCursor(0,0);
  //   display_handler.print("numSamples: ");
  //   display_handler.println(numSamples);


  //   display_handler.display();

  // delay(500);


    // display_handler.clearDisplay();
    // display_handler.setTextSize(1);
    // display_handler.setTextColor(SSD1306_WHITE);
    // display_handler.setCursor(0,0);
    // display_handler.print("after while");
    // display_handler.println(micros());

    // display_handler.print(startTime);
    // display_handler.print(" ");
    // display_handler.println(finishTime);

    // display_handler.print(startTime - finishTime);

    // display_handler.println();
    // display_handler.display();
    //delay(500);

  //digitalWrite(PB11, HIGH);




  double oneK[2* numSamples] = {0};
  double oneKCorr[numSamples] = {0};

  //double oneKT = (double) numSamples / (double) (startTime - finishTime);


  int dt = ( finishTime - startTime );
  double oneKT = (double) numSamples / ( (double) dt );

  for(int i = 0; i < 2 * numSamples;  i++) {
  
    oneK[i] = sin(i * TWO_PI / oneKT);
  
  }

    // display_handler.clearDisplay();
    // display_handler.println("here");

    // display_handler.display();


  // for (int i = 0; i < IRsignal.size(); i++){

  //   display_handler.clearDisplay();
  //   display_handler.setCursor(0,0);
  //   display_handler.print(i);
  //   display_handler.print(", ");
  //   display_handler.println(IRsignal.at(i));
  //   display_handler.display();
  //   delay(500);

  // }

  for (int k = 0; k < numSamples; k++){

    oneKCorr[k] = 0;

    for (int i = 0; i < numSamples; i++){
      
      //oneKCorr[k] = IRsignal.at(i);
      
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

    display_handler.clearDisplay();
    display_handler.setTextSize(1);
    display_handler.setTextColor(SSD1306_WHITE);
    display_handler.setCursor(0,0);
    // display_handler.print("startTime: ");
    // display_handler.println(startTime);

    // display_handler.print("finishTime: ");
    // display_handler.println(finishTime);



    // display_handler.print("dt: ");
    // display_handler.println(dt);

    // display_handler.print("numSamples: ");
    // display_handler.println(numSamples);

    // display_handler.print("oneKT: ");
    // display_handler.println(oneKT);

    display_handler.print("cc max: ");
    display_handler.println(max,0);

    display_handler.print("minTot: ");
    display_handler.println(minTot,0);

    display_handler.print("maxTot: ");
    display_handler.println(maxTot,0);

    display_handler.print("avg: ");
    display_handler.println(avg,0);

    display_handler.display();


   // if (max < -0.00)
      //delay(2500);

   // */

}

