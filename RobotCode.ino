
#include <ps5Controller.h>

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

const int cNumMotors = 2;
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
/*const float kp = 1.5;                     //don't need proportional gains or location feedback from wheel motors
const float ki = 0.2;
const float kd = 0.8;*/

const int in1 = 16;     
const int in2 = 17;     
const int in3 = 18;     
const int in4 = 19;   

//int speed1 = 0;                 //might implement speed if we change to using the joysticks
//int speed2 = 0;                 //using the joysticks would include relating the speed input range
                                  //to the joystick input range

//Encoder enconder[] = {{25, 26, 0}, {32, 33, 0}};

long target[] = {0, 0};             //probably won't need this section or the encoder line above
long lastEncoder[] = {0, 0};
float targetF[] = {0.0, 0.0}; 
int driveDir = 1;

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
  setUpPinModes();
  Serial.begin(115200);

  /*ledcAttach(in1, cPWMFreq, cPWMRes); //don't need this and the digitalWrite commands
  ledcAttach(in2, cPWMFreq, cPWMRes);
  ledcAttach(in3, cPWMFreq, cPWMRes);
  ledcAttach(in4, cPWMFreq, cPWMRes);*/

  //ps5.attach(notify);              //might need to use ps5.attach for the direction voids
  ps5.attachOnConnect(onConnect);
  ps5.attachOnDisconnect(onDisConnect);
  ps5.begin("D0:BC:C1:09:BF:B9");           //will have to change this based on the mac address of ps5 controller
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


void loop() {
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


  }if (ps5.Up() == HIGH) {
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
  }


  /*forward();  
  delay(1000);*/
}
