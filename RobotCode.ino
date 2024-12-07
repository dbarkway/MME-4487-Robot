//Title: Robot Rover Drive System
//Class: MME 4487a Mechatronic System Design
//Professor: Dr. Michael Naish
//By: David Barkway, Richard Fotevski, Mateen Khalatbari, Gareth Cooper
//Description: This code is to run our robot controlled by a playstation 5 controller. Coded to have precise direction control,
//             the ability to turn on a tight radius, sort beads based on color, store the green beads, and dump the green beads
//             into a bin at the end.


#include <ps5Controller.h>


const int cNumMotors = 2;                            //defining parameters for the motors
const int cPWMRes = 8;
const int cMinPWM = 0;
const int cMaxPWM = pow(2, cPWMRes) - 1;
const int cPWMFreq = 20000;
const int cCountsRev = 1096;
const int cMaxChange = 13;

const int in1 = 16;                                  //setting the pins of the gear motors
const int in2 = 17;     
const int in3 = 18;     
const int in4 = 19;   

int driveDir = 1;

void onConnect()                                    //checking that the controller is connected
{
  Serial.println("Connected!.");
}

void onDisConnect()
{
  Serial.println("Disconnected!.");    
}

void setUpPinModes()                                //setting the motor connections as outputs on the ESP32
{
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);      
}

void setup() 
{
  setUpPinModes();                                  //calling the void setting the connections as outputs
  Serial.begin(115200);

  ps5.attachOnConnect(onConnect);
  ps5.attachOnDisconnect(onDisConnect);
  ps5.begin("D0:BC:C1:09:BF:B9");                   //connecting the playstation controller through specific bluetooth mac address
  while (ps5.isConnected() == false) 
  { 
    Serial.println("PS5 controller not found");     
    delay(300);
  } 
  Serial.println("Ready.");
}

void stationary(){                                  //creating each directional movement for the robot by setting the pins high or low
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


void loop() {                                         //continuous process for the system
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
  }
  }
}
