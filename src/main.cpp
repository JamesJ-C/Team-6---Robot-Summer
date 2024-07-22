/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-two-way-communication-esp32/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <HardwareSerial.h>

#include <stack>

#include <robotConstants.h>


int callCount = 0;

#define RX 9
#define TX 10

HardwareSerial SerialPort(1);  //if using UART1

String received;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


// REPLACE WITH THE MAC Address of your receiver 
uint8_t broadcastAddress[] = {0x64, 0xb7, 0x08, 0x9d, 0x68, 0x0c};
//uint8_t broadcastAddress[] = {0x64, 0xb7, 0x08, 0x9c, 0x5c, 0xe0};

// Define variables to store readings to be sent

int reflectance1;
int reflectance2;
double transferFunction;
String strMsg;

// Define variables to store incoming readings

int incomingReflectance1;
int incomingReflectance2;
double incomingTransferFunction;
String incomingStrMsg;

// Variable to store if sending data was successful
String success;

typedef struct struct_message {

  int reflectance1;
  int reflectance2;
  double transferFunction;
  String strMsg;


} struct_message;

std::stack<String> incomingInfoStack;


// Create a struct_message called BME280Readings to hold sensor readings
struct_message msg;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success :)";
  }
  else{
    success = "Delivery Fail :(";
  }
}



// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  incomingReflectance1 = incomingReadings.reflectance1;
  incomingReflectance2 = incomingReadings.reflectance2;
  incomingTransferFunction = incomingReadings.transferFunction;
  incomingStrMsg = incomingReadings.strMsg;
  Serial.println("incoming msg: " + String(incomingReadings.strMsg));

  incomingInfoStack.push(incomingReadings.strMsg);

  if (callCount % 4 == 0){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
  }
  display.println(":" + String(incomingReadings.strMsg));
  display.display();
  callCount++;

  delay(1000);
}


void updateDisplay();
void getReadings();

void setup() {

  SerialPort.begin(115200, SERIAL_8N1, RX, TX);

  // Init Serial Monitor
  Serial.begin(115200);


  // // Init OLED display
  // if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
  //   Serial.println(F("SSD1306 allocation failed"));
  //   for(;;);
  // }

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
 

  display.display();
  delay(2000);

  // Displays "Hello world!" on the screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Setting up...");
  display.display();

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
int count = 1;

void loop() {
  
  getReadings();
 
  // Set values to send
  msg.reflectance1 = reflectance1;
  msg.reflectance2 = reflectance2;
  msg.transferFunction = transferFunction;
  msg.strMsg = strMsg;

  // Send message via ESP-NOW
  //esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &msg, sizeof(msg));
   
  // if (result == ESP_OK) {
  //   Serial.println("Sent with success");
  // }
  // else {
  //   Serial.println("Error sending the data");
  // }
  
  //Serial.println("incoming msg: " + String(incomingReadings.strMsg));
  //Serial.println("incoming msg: " + incomingReflectance1);
  // Serial.println("incoming msg: " + String(incomingStrMsg));
  // Serial.println();
  if(incomingReadings.strMsg.length() > 1){
    //delay(1000);
  }

  //updateDisplay();
  
  // if (count % 4 ==0) {
  //   delay(500);
  // }
  // count++;
}


void getReadings(){
  // temperature = -273;
  // humidity = 10.1;
  // pressure = 100000;


  // reflectance1 = -62;
  // reflectance1 = -16;
  // transferFunction = 699;
    //Serial.println("readings>>>");
    if (SerialPort.available() > 0) {

      //Serial.println("if sttmt");
      received = "";
      received = SerialPort.readStringUntil('\n');
      
      strMsg = received;
      //Serial.println("msg: " + String(received));

    } else {
      //Serial.println("else statmen");
      strMsg = "n/a: " + String(received);
    
    }



}

void updateDisplay(){
  // Display Readings on OLED Display
  // display.clearDisplay();
  // display.setTextSize(1);
  // display.setTextColor(WHITE);
  // display.setCursor(0, 0);
  // display.println("INCOMING READINGS");
  // display.setCursor(0, 15);
  // display.print("Temperature: ");
  // display.print(incomingTemp);
  // display.cp437(true);
  // display.write(248);
  // display.print("C");
  // display.setCursor(0, 25);
  // display.print("Humidity: ");
  // display.print(incomingHum);
  // display.print("%");
  // display.setCursor(0, 35);
  // display.print("Pressure: ");
  // display.print(incomingPres);
  // display.print("hPa");
  // display.setCursor(0, 56);
  // display.print(success);
  // display.display();
  
  // Display Readings in Serial Monitor
  // Serial.println("INCOMING READINGS");
  // Serial.print("ref1: ");
  // Serial.print(incomingReadings.reflectance1);
  // //Serial.println(" ºC");
  // Serial.print("ref2: ");
  // Serial.print(incomingReadings.reflectance2);
  // //Serial.println(" %");
  // Serial.print("transfer func: ");
  // Serial.print(incomingReadings.transferFunction);
  // Serial.print("strMsg: ");
  // Serial.print(incomingReadings.strMsg);
  // //Serial.println(" hPa");
  // Serial.println();
}