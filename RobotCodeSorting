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

/*struct Encoder {      //don't need position feedback from wheel motors
  const int chanA;
  const int chanB;
  long pos;
}*/

//void setMotor(int pwm, int in1, int in2, int in3, int in4); //used different voids

/*int   led1 = 15;
int   led2 = 16;      //not using leds
int   led3 = 17;
int   led4 = 18;
int   led5 = 19;
int   led6 = 21;
int   led7 = 22;
int   led8 = 23;     */

//const int cNumMotors = 2;
//const int cIN1Pin = 16;
//const int cIN2Pin = 17;
//const int cIN3Pin = 18;
//const int cIN4Pin = 19;
const int cPWMRes = 8;
const int cMinPWM = 0;
const int cMaxPWM = pow(2, cPWMRes) - 1;
const int cPWMFreq = 20000;
const int cCountsRev = 1096;
const int cMaxChange = 13;

const int cTCSLED = 23;


/*const float kp = 1.5;                     //don't need proportional gains or location feedback from wheel motors
const float ki = 0.2;
const float kd = 0.8;*/

const int in1 = 16;     
const int in2 = 17;     
const int in3 = 18;     
const int in4 = 19;   
const int cServoPin1 = 15;
const int cServoPin2 = 4;
const int cServoPin3 = 2;
const long cMinDutyCycle = 1650;      //1650
const long cMaxDutyCycle = 8175;      //8175

long degreesToDutyCycle(int deg) {
  return map(deg, 0, 180, cMinDutyCycle, cMaxDutyCycle);
}

//int speed1 = 0;                 //might implement speed if we change to using the joysticks
//int speed2 = 0;                 //using the joysticks would include relating the speed input range
                                  //to the joystick input range

//Encoder enconder[] = {{25, 26, 0}, {32, 33, 0}};

long target[] = {0, 0};             //probably won't need this section or the encoder line above
long lastEncoder[] = {0, 0};
float targetF[] = {0.0, 0.0}; 
int driveDir = 1;
int servoPos1;
int servoPos2;
int servoPos3;

int colordiff;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

//uint16_t r, g, b, c;

/*void notify()
{
  digitalWrite(in1, ps5.Up());        //needed to connect direction of the robot to ps5 buttons
  digitalWrite(in2, ps5.Down());      //not ps5 buttons to specific inputs
  digitalWrite(in3, ps5.Left());
  digitalWrite(in4, ps5.Right());     //future note: could change it to this if needed, just harder to control

  /*digitalWrite(led1, ps5.Left());
  digitalWrite(led2, ps5.Down());
  digitalWrite(led3, ps5.Right());
  digitalWrite(led4, ps5.Up()); 

  digitalWrite(led5, ps5.Square());
  digitalWrite(led6, ps5.Cross());
  digitalWrite(led7, ps5.Circle());
  digitalWrite(led8, ps5.Triangle()); 
}*/

void onConnect()
{
  Serial.println("Connected!.");
}

void onDisConnect()
{
  Serial.println("Disconnected!.");    
}

void setUpPinModes()
{
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);      

  /*pinMode(led5, OUTPUT);
  pinMode(led6, OUTPUT);
  pinMode(led7, OUTPUT);
  pinMode(led8, OUTPUT); */
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


  /*ledcAttach(in1, cPWMFreq, cPWMRes); //don't need this and the digitalWrite commands
  ledcAttach(in2, cPWMFreq, cPWMRes);
  ledcAttach(in3, cPWMFreq, cPWMRes);
  ledcAttach(in4, cPWMFreq, cPWMRes);*/

  //ps5.attach(notify);              //might need to use ps5.attach for the direction voids
  ps5.attachOnConnect(onConnect);
  ps5.attachOnDisconnect(onDisConnect);
  ps5.begin("D0:BC:C1:09:BF:B9");           //will have to change this based on the mac address of ps5 controller
                                            //Ronins: 24A6FAE40DC7    Mateen's: D0BCC109BFB9
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
  servoPos1 = 100;
  ledcWrite(cServoPin1, degreesToDutyCycle(servoPos1));
}

void droparm() {
  servoPos1 = 0;
  ledcWrite(cServoPin1, degreesToDutyCycle(servoPos1));
}

/*void read() {
  uint16_t r, g, b, c;
  colordiff = r - g;
}*/

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

void loop() {
  uint16_t r, g, b, c;
  /*while (tcsFlag = true) {                                      // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
#ifdef PRINT_COLOUR            
    Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
#endif
  }*/

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
    //tcsFlag = true;
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
    //colordiff = r - g;
    /*if (g < 70) {
      green();
    }
    else {
      red();
    }*/
  }
  if (ps5.L2()) {
    dropBeads();
  }
  if (ps5.Touchpad()) {
    flat();
    flat2();
  }
  /*if (ps5.R2()) {
    dropBeads();
  }
  if (ps5.L2()) {
    flat2();
  }
  if (ps5.R1()) {
    green();
  }
  if(ps5.L1()) {
    red();
  }
  if(ps5.Circle()){
    flat();
  }*/


  }/*if (ps5.Up() == HIGH) {
    forward();
  }
   if (ps5.Down() == HIGH) {
    backward();
  }
  if (ps5.Left() == HIGH) {
    left();
  }
  if (ps5.Right() == HIGH) {
  right();
}
  if (ps5.Square() == HIGH){
    stationary();
  }
  if(ps5.Triangle() == HIGH){
    pickup();
  }
  if (ps5.Cross() ==  HIGH) {
    droparm();
  }
  if (ps5.R2() == HIGH) {
    colordiff = r - g;
    if (colordiff >= 3) {
      green();
    }
    else {
      red();
    }
  }
  if (ps5.L2() == HIGH) {
    dropBeads();
  }
  if (ps5.R1() == HIGH) {
    flat();
    flat2();
  }*/
  /*if (ps5.R2() == HIGH) {
    dropBeads();
  }
  if (ps5.L2() == HIGH) {
    flat2();
  }
  if (ps5.R1() == HIGH) {
    green();
  }
  if(ps5.L1() == HIGH) {
    red();
  }
  if(ps5.Circle() == HIGH){
    flat();
  }*/
  }


  /*forward();  
  delay(1000);*/
}
