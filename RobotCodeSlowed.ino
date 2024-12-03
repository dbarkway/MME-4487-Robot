//Title: Robot Rover and Bead Sorting Code
//Class: MME 4487a Mechatronic System Design
//Professor: Dr. Michael Naish
//By: David Barkway, Richard Fotevski, Mateen Khalatbari, Gareth Cooper
//Description: This code is to run our robot controlled by a playstation 5 controller. Coded to have precise direction control,
//             the ability to turn on a tight radius, sort beads based on color, store the green beads, and dump the green beads
//             into a bin at the end.


#define OUTPUT_ON

#include <ps5.h>
#include <ps5Controller.h>
#include <ps5_int.h>

#include <ps5Controller.h>
#include <Arduino.h>

#define PRINT_COLOUR                                  // uncomment to turn on output of colour sensor data

#include <Adafruit_TCS34725.h>
#include <Wire.h>
#include <SPI.h>

const int cPWMRes = 8;
const int cMinPWM = 0;
const int cMaxPWM = pow(2, cPWMRes) - 1;
const int cPWMFreq = 20000;

const int cTCSLED = 23;

const int in1 = 16;     
const int in2 = 17;     
const int in3 = 18;     
const int in4 = 19;   
const int cServoPin1 = 15;
const int cServoPin2 = 4;
const int cServoPin3 = 2;
const long cMinDutyCycle = 1650;      //1650
const long cMaxDutyCycle = 8175;      //8175

int servoSpeedDelay = 10;  // Speed control delay value, lower means faster movement
int targetServoPos1 = 0;
int currentServoPos1 = 0;

long degreesToDutyCycle(int deg) {
  return map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);
}

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

void onConnect()
{
  Serial.println("Connected!.\n");
}

void onDisConnect()
{
  Serial.println("Disconnected!.\n");    
}

void setUpPinModes()
{
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);      
}

void setup() 
{
  #ifdef OUTPUT_ON
  Serial.begin(115200);
  #endif
  
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO for control of LED on TCS34725
  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
    digitalWrite(cTCSLED, 1);                         // turn on onboard LED 
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }

  setUpPinModes();
  ledcAttach(cServoPin1, 50, 16);
  ledcAttach(cServoPin2, 50, 16);
  ledcAttach(cServoPin3, 50, 16);

  ps5.attachOnConnect(onConnect);
  ps5.attachOnDisconnect(onDisConnect);
  ps5.begin("D0:BC:C1:09:BF:B9");           // MAC address of PS5 controller
  while (ps5.isConnected() == false) 
  { 
    Serial.println("PS5 controller not found");
    delay(300);
  } 
  Serial.println("Ready.");
}

void stationary(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void left() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void forward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void backward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void right() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void pickup() {
  targetServoPos1 = 100;
}

void droparm() {
  targetServoPos1 = 0;
}

void green() {
  int servoPos2 = 0;
  ledcWrite(cServoPin2, degreesToDutyCycle(servoPos2));
}

void red() {
  int servoPos2 = 120;
  ledcWrite(cServoPin2, degreesToDutyCycle(servoPos2));
}

void flat() {
  int servoPos2 = 60;
  ledcWrite(cServoPin2, degreesToDutyCycle(servoPos2));
}

void flat2() {
  int servoPos3 = 60;
  ledcWrite(cServoPin3, degreesToDutyCycle(servoPos3));
}

void dropBeads() {
  int servoPos3 = 0;
  ledcWrite(cServoPin3, degreesToDutyCycle(servoPos3));
}

void updateServoPosition() {
  if (currentServoPos1 < targetServoPos1) {
    currentServoPos1++;
  } else if (currentServoPos1 > targetServoPos1) {
    currentServoPos1--;
  }
  ledcWrite(cServoPin1, degreesToDutyCycle(currentServoPos1));
  delay(servoSpeedDelay);  // Adjust speed by changing this value
}

void loop() {
  uint16_t r, g, b, c;
  
  if (ps5.isConnected() == true){
    while (ps5.isConnected() == true){

      if (ps5.Up()) {
        forward();
      }
      if (ps5.Down()) {
        backward();
      }
      if (ps5.Left()) {
        left();
      }
      if (ps5.Right()) {
        right();
      }
      if (ps5.Square()) {
        stationary();
      }
      if (ps5.Triangle()) {
        pickup();
      }
      if (ps5.Cross()) {
        droparm();
      }
      if (ps5.R2()) {
        if (tcsFlag) {                                      // if colour sensor initialized
          tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
          if (c < 200) {
            green();
          }
          else {
            red();
          }
#ifdef PRINT_COLOUR            
          Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
#endif
        }
      }
      if (ps5.L2()) {
        dropBeads();
      }
      if (ps5.Touchpad()) {
        flat();
        flat2();
      }
      updateServoPosition(); // Update servo 1 position gradually
    }
  }
}
