//Title: Robot Rover and Bead Sorting Code
//Class: MME 4487a Mechatronic System Design
//Professor: Dr. Michael Naish
//By: David Barkway, Richard Fotevski, Mateen Khalatbari, Gareth Cooper
//Description: This code is to add color reading and sorting onto the robot drive system code


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
const int cCountsRev = 1096;
const int cMaxChange = 13;

const int cTCSLED = 23;


const int in1 = 16;                                      //setting the pins of the gear motors     
const int in2 = 17;     
const int in3 = 18;     
const int in4 = 19;   
const int cServoPin1 = 15;                               //setting the pins for the servo motors
const int cServoPin2 = 4;
const int cServoPin3 = 2;
const long cMinDutyCycle = 1650;                         //the dutycycle of the servo motor to convert it into degrees
const long cMaxDutyCycle = 8175;  

long degreesToDutyCycle(int deg) {                       //comparing 0 to 180 degrees to the duty cycle of the servos to have an angle input  
  return map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);
}


int driveDir = 1;
int servoPos1;
int servoPos2;
int servoPos3;

int colordiff;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                         //TCS34725 flag: 1 = connected; 0 = not found
//uint16_t r, g, b, c;


void onConnect()                                           //checking that the controller is connected
{
  Serial.println("Connected!.");
}

void onDisConnect()
{
  Serial.println("Disconnected!.");    
}

void setUpPinModes()                                       //setting the motor connections as outputs on the ESP32
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
  

  pinMode(cTCSLED, OUTPUT);                                //configure GPIO for control of LED on TCS34725
  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
    digitalWrite(cTCSLED, 1);                             //turn on onboard LED 
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }

  setUpPinModes();                                         //calling the void setting the connections as outputs
  ledcAttach(cServoPin1, 50, 16);
  ledcAttach(cServoPin2, 50, 16);
  ledcAttach(cServoPin3, 50, 16);


  ps5.attachOnConnect(onConnect);
  ps5.attachOnDisconnect(onDisConnect);
  ps5.begin("D0:BC:C1:09:BF:B9");                         //connecting the playstation controller through specific bluetooth mac address
                                                          //Two controllers we used, Ronins: 24A6FAE40DC7    Mateen's: D0BCC109BFB9
  while (ps5.isConnected() == false) 
  { 
    Serial.println("PS5 controller not found");
    delay(300);
  } 
  Serial.println("Ready.");
}

void stationary(){                                        //creating each directional movement for the robot by setting the pins high or low
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
void pickup() {                                                //creating other functions for the motors in the system
  servoPos1 = 100;                                             //setting the servo motors to different angles
  ledcWrite(cServoPin1, degreesToDutyCycle(servoPos1));
}

void droparm() {
  servoPos1 = 0;
  ledcWrite(cServoPin1, degreesToDutyCycle(servoPos1));
}

void green() {
  servoPos2 = 0;
  ledcWrite(cServoPin2, degreesToDutyCycle(servoPos2));
}

void red() {
  servoPos2 = 120;
  ledcWrite(cServoPin2, degreesToDutyCycle(servoPos2));
}

void flat() {
  servoPos2 = 60;
  ledcWrite(cServoPin2, degreesToDutyCycle(servoPos2));
}

void flat2() {
  servoPos3 = 60;
  ledcWrite(cServoPin3, degreesToDutyCycle(servoPos3));
}

void dropBeads() {
  servoPos3 = 0;
  ledcWrite(cServoPin3, degreesToDutyCycle(servoPos3));
}

void loop() {                                             //continuous process for the system
  uint16_t r, g, b, c;

  if (ps5.isConnected() == true){
    while (ps5.isConnected() == true){

      if (ps5.Up()) {                                     //connecting buttons on the controller to the functions
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
    //tcsFlag = true;
    if (tcsFlag) {                                      //if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                     //get raw RGBC values
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
  }
  }
}
