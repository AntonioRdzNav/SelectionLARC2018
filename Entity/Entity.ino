/////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////SELECCION LARC////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

#include <StackArray.h>
#include <math.h>
///////////////////////////////////////////////COLOR///////////////////////////////////////////////////
// AT
// #define sensorOut 32
// #define S2 30
// #define S3 28
// #define S1 38
// #define S0 36
// #define OE 40
// Seleccion LARC
#define sensorOut 13
#define S2 12
#define S3 11
#define S1 10
#define S0 8

bool colorRedDetected=false, colorGreenDetected=false, colorBlackDetected=false;
double r = 0, g = 0, b = 0;
const int num_col = 5;
int range = 20;
int color_position;           //  0        1       2       3        4
String color_names[5] = {"rojo", "azul", "verde", "negro", "blanco"};
bool switchColor=false;
int initialColors[3];
struct color{
  String nombre;
  double red;
  double green;
  double blue;
};
color color_position_arr[num_col];

//////////////////////////////////////////////LEDs////////////////////////////////////////////////
//AT
#define ledRed 7
#define ledGreen 4
#define ledBlue 2

/////////////////////////////////////////////MOTORS///////////////////////////////////////////////
// AT
// #define motorL1 5  //Back
// #define motorL2 3  //Front
// #define motorR1 9   //Back
// #define motorR2 6  //Front
// #define motorL_PWM 7  //PWM
// #define motorD_PWM 4  //PWM
// Seleccion LARC
#define motorL1 5 //Back
#define motorL2 3  //Front
#define motorR1 9 //Back
#define motorR2 6  //Front

double velGenDer = 90;
double velGenIzq = 90;
double velGenDerWall = 50;
double velGenIzqWall = 50;
double velGenDerBack = 100;
double velGenIzqBack = 100;
//AT
double timeStepBack=1100, timeStep=920;
//PISTA LAB
//double timeStepBack=1250, timeStep=1080;
double maxTurnVel=150;
double maxFrontVel=255;
double maxWallVel=255;
bool spinRight, spinLeft;

///////////////////////////////////////////ULTRASONICS////////////////////////////////////////////
//#include <NewPing.h>
////AT
// #define echoRight 25
// #define trigRight 23
// #define echoLeft 41
// #define trigLeft 39
// #define echoFront 31
// #define trigFront 29
// #define echoBack 10
// #define trigBack 11
//bool special, ultraNegativoSide;
//double stepDistance = 27, backStepDistance = 35;
//bool setFakeSetpoint;
//bool firstBack=true;
//double MAX_DISTANCE = 250;  //Prevents from waiting too long on pulseIn()
//NewPing pingFront(trigFront, echoFront, MAX_DISTANCE);
//NewPing pingRight(trigRight, echoRight, MAX_DISTANCE);
//NewPing pingLeft(trigLeft, echoLeft, MAX_DISTANCE);
//NewPing pingBack(trigBack, echoBack, MAX_DISTANCE);

/////////////////////////////////////////////SHARP////////////////////////////////////////////////
#define sharpLeftPin A1
#define sharpRightPin A0
#define sharpFrontPin A2

///////////////////////////////////////////BNO///////////////////////////////////////////////////
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
sensors_event_t event; 

///////////////////////////////////////////IMU///////////////////////////////////////////////////
//#include <MPU6050.h>
//MPU6050 mpu;
//Vector rawGyro;
//Vector normGyro;

///////////////////////////////////////////PID BNO///////////////////////////////////////////////////
#include <utility/imumaths.h>
#include <PID_v1.h>
double leftAlignKp=37, leftAlignKi=0, leftAlignKd=0;
double leftTurnKp=10, leftTurnKi=0, leftTurnKd=0;
double leftConsKp=47, leftConsKi=0, leftConsKd=0;
double leftGenKp=leftConsKp, leftGenKi=leftConsKi, leftGenKd=leftConsKd;
double leftError=0;
double rightAlignKp=37, rightAlignKi=0, rightAlignKd=0;
double rightTurnKp= 10, rightTurnKi=0, rightTurnKd=0;
double rightConsKp=47, rightConsKi=0, rightConsKd=0;
double rightGenKp=rightConsKp, rightGenKi=rightConsKi, rightGenKd=rightConsKd;
double rightError=0;
double Offset, Setpoint, fakeSetpoint, leftOutput, rightOutput, Input, rawInput, fakeInput, lastSetpoint;
double outputDifference = 5;
double offsetAngle = 0.0000;
PID leftPID(&fakeInput, &leftOutput, &fakeSetpoint, leftGenKp, leftGenKi, leftGenKd, DIRECT); // (Values>0)
PID rightPID(&fakeInput, &rightOutput, &fakeSetpoint, rightGenKp, rightGenKi, rightGenKd, REVERSE); // (Values<0)
bool LARC = true;

///////////////////////////////////////////PID Sharp///////////////////////////////////////////////////
double sharpConsKp=8, sharpConsKi=0, sharpConsKd=0;
double SetpointWallDistance = 8, leftWallOutput, rightWallOutput, leftWheelOutput, rightWheelOutput;
double leftWallError, rightWallError;

///////////////////////////////////////////SHARP KALMAN FILTER///////////////////////////////////////////
struct SharpKalman{
  SharpKalman(){
    varSensor = 8e-6; //Variance of sensor. The LESS, the MORE it looks like the raw input.
    varProcess = 1e-7; 
    P = 1.0;
    Pc = 0.0;
    G = 0.0;
    Xp = 0.0;
    Zp = 0.0;
    kalmanDistance = 0.0;
    rawDistance = 0.0;
    distance = 0.0;
    side = false;
  }
  float varSensor; //Variance of sensor. The LESS, the MORE it looks like the raw input
  float varProcess; // The greater the variance, faster the sensor response
  float P;
  float Pc;
  float G;
  float Xp;
  float Zp;
  float kalmanDistance;
  double rawDistance;
  double distance;
  bool side;
};
SharpKalman sharpRight;
SharpKalman sharpLeft;
SharpKalman sharpFront;

///////////////////////////////////////////ULTRA KALMAN FILTER///////////////////////////////////////////
//struct UltraKalman{
//  UltraKalman(){
//    varSensor = 3e-6; //Variance of sensor. The LESS, the MORE it looks like the raw input.
//    varProcess = 1e-7; 
//    P = 1.0;
//    Pc = 0.0;
//    G = 0.0;
//    Xp = 0.0;
//    Zp = 0.0;
//    kalmanDistance = 0.0;
//    duration = 0.0;
//    distance = 0.0;
//    side = false;
//  }
//  float varSensor; //Variance of sensor. The LESS, the MORE it looks like the raw input
//  float varProcess; // The greater the variance, faster the sensor response
//  float P;
//  float Pc;
//  float G;
//  float Xp;
//  float Zp;
//  float kalmanDistance;
//  double duration;
//  double distance;
//  bool side;
//};
//UltraKalman ultraRight;
//UltraKalman ultraLeft;
//UltraKalman ultraFront;
//UltraKalman ultraBack;

///////////////////////////////////////////xBNO KALMAN FILTER///////////////////////////////////////////
float xBNOVarSensor = 10e-6; //Variance of sensor. The LESS, the MORE it looks like the raw input
float xBNOVarProcess = 1e-7; // The greater the variance, faster the sensor response
float xBNOP = 1.0;
float xBNOPc = 0.0;
float xBNOG = 0.0;
float xBNOXp = 0.0;
float xBNOZp = 0.0;
float xBNOXe = 0.0; // Xe

///////////////////////////////////////////xIMU KALMAN FILTER///////////////////////////////////////////
//float xIMUVarSensor = 10e-4; //Variance of sensor. The LESS, the MORE it looks like the raw input
//float xIMUVarProcess = 1e-7; // The greater the variance, faster the sensor response
//float xIMUP = 1.0;
//float xIMUPc = 0.0;
//float xIMUG = 0.0;
//float xIMUXp = 0.0;
//float xIMUZp = 0.0;
//float xIMUXe = 0.0; // Xe

/////////////////////////////////////////////SETUP////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
//  Serial.begin(115200);
  pinMode(motorR1,OUTPUT);
  pinMode(motorR2,OUTPUT);
  pinMode(motorL1,OUTPUT);
  pinMode(motorL2,OUTPUT);  
  pinMode(ledGreen,OUTPUT);
  pinMode(ledRed,OUTPUT);
  pinMode(ledBlue,OUTPUT);    
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
//  pinMode(limitSwitchDer,INPUT);
//  pinMode(limitSwitchIzq,INPUT);  
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);   
//// BNO
//////////////////////////////////////////////////////////////////////////////////////////////////
//  calibrarColores(1);
//  hardCodedCalibration();
//  turnOffLeds();
//  delay(5000);
//////////////////////////////////////////////////////////////////////////////////////////////////
  if(!bno.begin())  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or L2C ADDR!");
    while(1);
  } 
  else
    Serial.println("BNO STARTED");
//  setCal();
//  delay(3000);
  bno.setExtCrystalUse(true);
//// PID
  Setpoint=0; //  Set reference point to 0
  leftPID.SetSampleTime(1); // Set Sample Time
  leftPID.SetMode(AUTOMATIC);
  leftPID.SetOutputLimits(0, 255); 
  rightPID.SetSampleTime(1); // Set Sample Time
  rightPID.SetMode(AUTOMATIC);
  rightPID.SetOutputLimits(0, 255);
  delay(500);
}

void loop(){
//   filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
//   if(ultraNegativoSide){
//       spinPID(-180, false);
//       stop(false);
//       delay(1000);
//       filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);      
//   }

//  rightPriotity(ultraFront, ultraRight, ultraLeft); 
//  firstControlChallenge();
//  secondControlChallenge();
  thirdControlChallenge();
//  currentColor();

//  forwardPID();
//  Serial.println(rawInput);

//  filtrateDistances(ultraFront, ultraRight, ultraLeft);
//  spinPID(90);
//  backPID(1000);

//go();

//calculateRawDistancesSharp();
//  Serial.print(sharpLeft.distance);
//  Serial.print("\t");
//  Serial.print(sharpFront.distance);
//  Serial.print("\t");
//  Serial.println(sharpRight.distance);
//  delay(5);

//  oneStepMillis(true);
//  stop(true);
//  delay(2000);

//  calibrarColores(1);
//  while(1){
//    readColor(r, g, b);
//    Serial.print(r);
//    Serial.print("\t");
//    Serial.print(g);
//    Serial.print("\t");
//    Serial.println(b); 
//    Serial.println(currentColor());
//  } 

//  xBNO_RawKalman(bno, event);
//  sharp_RawKalman(sharpLeft);
//  sharp_RawKalman(sharpFront);
//  sharp_RawKalman(sharpRight);
//  ultra_RawKalman(ultraFront, pingFront);
//  ultra_RawKalman(ultraLeft, pingLeft);
//  ultra_RawKalman(ultraRight, pingRight);
//  ultra_RawKalman(ultraBack, pingBack);
}
