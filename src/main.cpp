#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <robotConstants.h>

#include <queue>

#include <Motor.h>
#include <RotaryEncoder.h>


int callCount = 0;

#define RX 9
#define TX 10

HardwareSerial SerialPort(1);  //if using UART1

String received;

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


// REPLACE WITH THE MAC Address of your receiver 
//uint8_t broadcastAddress[] = {0x64, 0xb7, 0x08, 0x9d, 0x68, 0x0c};
uint8_t broadcastAddress[] = {0x64, 0xb7, 0x08, 0x9c, 0x5c, 0xe0};

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

std::queue<String> incomingUARTInfoQueue;

std::queue<String> incomingWifiInfoQueue;


// Create a struct_message called BME280Readings to hold sensor readings
struct_message msg;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;

esp_now_peer_info_t peerInfo;


void setDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
}


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
  // // // Serial.println("incoming msg: " + String(incomingReadings.strMsg));

  // setDisplay();
  // display.println(incomingReadings.strMsg);
  // display.display();

  incomingWifiInfoQueue.push(incomingReadings.strMsg);

}

void updateDisplay();
void getReadings();

void ISRUpdateElevatorEncoder();
void ISRUpdateArmEncoder();
void ISRLazySusanLimitSwitch();



/*  Object Declerations  */


encoder::RotaryEncoder elevatorEncoder(ELEVATOR_ROTARY_A, ELEVATOR_ROTARY_B);
encoder::RotaryEncoder armEncoder(ARM_ROTARY_A, ARM_ROTARY_B);

movement::Motor elevatorMotor(MOTOR_1_a, MOTOR_1_b, "elevator", 1, 2, &elevatorEncoder);
movement::Motor armMotor(MOTOR_2_a, MOTOR_2_b, "arm", 3, 4, &armEncoder);

void setup() {


/*  Serial Setup  */
{
  SerialPort.begin(115200, SERIAL_8N1, RX, TX);

  // Init Serial Monitor
  Serial.begin(115200);
  Serial.println("Setup");
}

/*  Display setup  */
{
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

}

/*  Wifi stuff  */
  {

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    setDisplay();
    display.println("Error initializing ESP-NOW");
    display.display();
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


}//wifi


/*  Interrupt Attaches  */
{
  attachInterrupt(digitalPinToInterrupt(ELEVATOR_ROTARY_A), ISRUpdateElevatorEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ELEVATOR_ROTARY_B), ISRUpdateElevatorEncoder, CHANGE);

  attachInterrupt(digitalPinToInterrupt(ARM_ROTARY_A), ISRUpdateArmEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ARM_ROTARY_B), ISRUpdateArmEncoder, CHANGE);


  uint8_t n = 19;//5, 37 weird
  pinMode(n, INPUT);
  attachInterrupt(digitalPinToInterrupt(n), ISRLazySusanLimitSwitch, CHANGE);
}


pinMode(elevatorMotor.getPinA(), OUTPUT);
pinMode(elevatorMotor.getPinB(), OUTPUT);

pinMode(armMotor.getPinA(), OUTPUT);
pinMode(armMotor.getPinB(), OUTPUT);


// ledcAttachPin(elevatorMotor.getPinA(), 1);
// ledcAttachPin(elevatorMotor.getPinB(), 2);
// ledcAttachPin(armMotor.getPinA(), 3);
// ledcAttachPin(armMotor.getPinB(), 4);
// // ledcAttachPin(5, 5);
// ledcSetup(1, 500, 12);
// ledcSetup(2, 500, 12);
// ledcSetup(3, 500, 12);
// ledcSetup(4, 500, 12);
// ledcSetup(5, 500, 12);
// ledcAttachPin(MOTOR_1_a, 1);
// ledcAttachPin(MOTOR_1_b, 2);
// ledcAttachPin(elevatorMotor.getPinA(), 1);
// ledcAttachPin(elevatorMotor.getPinB(), 2);
// ledcSetup(1, 1000, 8);
// ledcSetup(2, 1000, 8);
// ledcAttachPin(MOTOR_2_a, 3);
// ledcAttachPin(MOTOR_2_b, 4);
// ledcAttachPin(armMotor.getPinA(), 3);
// ledcAttachPin(armMotor.getPinB(), 4);
// ledcSetup(3, 1000, 8);
// ledcSetup(4, 1000, 8);


}


int uartItemsDisplayed = 0;
int wifiItemsDisplayed = 0;

bool button = true;
bool prevButton = true;

void loop() {


if (button != prevButton) {
  
  Serial.println("button Pressed");
  setDisplay();
  display.println("lazySusan");
  display.display();
  msg.strMsg = "button";
  prevButton = button;
} else {
  prevButton = button;
}

// ledcWrite(1, 200);
// ledcWrite(2, 200);
// ledcWrite(3, 200);
// ledcWrite(4, 200);

  elevatorMotor.forward(400);
  
  armMotor.forward(400);
  // ledcWrite(5, 500);

/*  Wifi  */
{
  getReadings();
 
  // Set values to send
  msg.reflectance1 = reflectance1;
  msg.reflectance2 = reflectance2;
  msg.transferFunction = transferFunction;
  msg.strMsg = strMsg;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &msg, sizeof(msg));
   
  if (result == ESP_OK) {
    // / // /Serial.println("Sent with success");
  }
  else {
    // // / // Serial.println("Error sending the data");
  }
  
  updateDisplay();
  //Serial.println("incoming msg: " + String(incomingReadings.strMsg));
  //Serial.println("incoming msg: " + incomingReflectance1);
  // Serial.println("incoming msg: " + String(incomingStrMsg));
  // Serial.println();

}



}


void getReadings(){
  
    if (SerialPort.available() > 0) {

      received = "";
      received = SerialPort.readStringUntil('\n');
      
      strMsg = received;
      incomingUARTInfoQueue.push(received);
      // display.println("msg: " + String(received));
      // display.println("msg: " + String(received));

    } else {
      //Serial.println("else statmen");
      strMsg = "n/a: " + String(received);
    
    }

//strMsg = "2." + received;

}

void updateDisplay(){

  //Serial.println("wifiQ size: " + String( incomingWifiInfoQueue.size() ) );
  //Serial.println("wifi items displayed: " + String( wifiItemsDisplayed ) );

  if (!incomingWifiInfoQueue.empty() && wifiItemsDisplayed < 3) {
    // Serial.println("1st if");
      display.println( "Wifi: " + incomingWifiInfoQueue.front() );
      // Serial.println( "Wifi: " + incomingWifiInfoQueue.front() );
      incomingWifiInfoQueue.pop();
      wifiItemsDisplayed++;
    } 

  if (!incomingUARTInfoQueue.empty() && uartItemsDisplayed < 3) {
      display.println( "UART: " + incomingUARTInfoQueue.front() );
      // Serial.println( "UART: " + incomingUARTInfoQueue.front() );
      incomingUARTInfoQueue.pop();
      uartItemsDisplayed++;
    } 

  if (uartItemsDisplayed >= 3 || wifiItemsDisplayed >= 3) {
      // Serial.println("disp if");
      wifiItemsDisplayed = uartItemsDisplayed = 0;
      display.display();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
  }

}


void IRAM_ATTR ISRUpdateElevatorEncoder(){

  bool A = digitalRead(elevatorEncoder.getPinA());
  bool B = digitalRead(elevatorEncoder.getPinB());

  elevatorEncoder.updateEncoder(A, B);

}

void IRAM_ATTR ISRUpdateArmEncoder() {


  bool A = digitalRead(armEncoder.getPinA());
  bool B = digitalRead(armEncoder.getPinB());

  armEncoder.updateEncoder(A, B);

}

void IRAM_ATTR ISRLazySusanLimitSwitch() {
  SerialPort.println("lazySusan");
  button = !button;

}