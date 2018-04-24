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
String color_names[num_col] = {"rojo", "azul", "verde", "negro", "blanco"};
bool switchColor=false;
struct color{
  String nombre;
  double red;
  double green;
  double blue;
};
color color_position_arr[num_col];

//////////////////////////////////////////////LCD////////////////////////////////////////////////
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
int lastMillis, actualMillis;

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


double velGenDer = 110;
double velGenIzq = 110;
double velGenDerBack = 110;
double velGenIzqBack = 110;
//AT
double timeStepBack=1100, timeStep=920;
//PISTA LAB
//double timeStepBack=1250, timeStep=1080;
double maxTurnVel=190;
double maxFrontVel=255;
bool spinRight, spinLeft;

/////////////////////////////////////////////LIMIT SWITCHES///////////////////////////////////////////////
//#define limitSwitchIzq A9
//#define limitSwitchDer A3
int turnsCounter = 0;
/////////////////////////////////////////////ENCODERS///////////////////////////////////////////////
//AT
//#define encoderM1Front 13
//#define encoderM1Back 12
//#define encoderM2Front A13
//#define encoderM2Back A12
//#define encoderM3Front 11
//#define encoderM3Back 10
//#define encoderM4Front A15
//#define encoderM4Back A14

///////////////////////////////////////////ULTRASONICS////////////////////////////////////////////
#include <NewPing.h>
//AT
 #define echoRight 25
 #define trigRight 23
 #define echoLeft 41
 #define trigLeft 39
 #define echoFront 31
 #define trigFront 29
 #define echoBack 10
 #define trigBack 11

bool special, ultraNegativoSide;
double stepDistance = 27, backStepDistance = 35;
bool setFakeSetpoint;
bool firstBack=true;
double MAX_DISTANCE = 250;  //Prevents from waiting too long on pulseIn()
NewPing pingFront(trigFront, echoFront, MAX_DISTANCE);
NewPing pingRight(trigRight, echoRight, MAX_DISTANCE);
NewPing pingLeft(trigLeft, echoLeft, MAX_DISTANCE);
NewPing pingBack(trigBack, echoBack, MAX_DISTANCE);

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
#include <MPU6050.h>
MPU6050 mpu;
Vector rawGyro;
Vector normGyro;

///////////////////////////////////////////PID///////////////////////////////////////////////////
#include <utility/imumaths.h>
#include <PID_v1.h>
double leftAlignKp=37, leftAlignKi=0, leftAlignKd=0;
double leftTurnKp=15, leftTurnKi=0, leftTurnKd=0;
double leftConsKp=38, leftConsKi=0, leftConsKd=0;
double leftGenKp=leftConsKp, leftGenKi=leftConsKi, leftGenKd=leftConsKd;
double leftError=0;
double rightAlignKp=37, rightAlignKi=0, rightAlignKd=0;
double rightTurnKp= 15, rightTurnKi=0, rightTurnKd=0;
double rightConsKp=38, rightConsKi=0, rightConsKd=0;
double rightGenKp=rightConsKp, rightGenKi=rightConsKi, rightGenKd=rightConsKd;
double rightError=0;
double Offset, Setpoint, fakeSetpoint, leftOutput, rightOutput, Input, rawInput, fakeInput, lastSetpoint;
double outputDifference = 5;
double offsetAngle = 0.2;
PID leftPID(&fakeInput, &leftOutput, &fakeSetpoint, leftGenKp, leftGenKi, leftGenKd, DIRECT); // (Values>0)
PID rightPID(&fakeInput, &rightOutput, &fakeSetpoint, rightGenKp, rightGenKi, rightGenKd, REVERSE); // (Values<0)
bool LARC = true;

//////////////////////////////////////////////ALGORITHM/////////////////////////////////////////////////
// const int mazeSize=9;
// int actualRow=0, actualCol=0;
// char comeFrom='S';
// String maze[mazeSize][mazeSize];

///////////////////////////////////////////ULTRA KALMAN FILTER///////////////////////////////////////////
struct UltraKalman{
  UltraKalman(){
    varSensor = 3e-6; //Variance of sensor. The LESS, the MORE it looks like the raw input.
    varProcess = 1e-7; 
    P = 1.0;
    Pc = 0.0;
    G = 0.0;
    Xp = 0.0;
    Zp = 0.0;
    Xe = 0.0;
    duration = 0.0;
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
  float Xe;
  double duration;
  double distance;
  bool side;
};
UltraKalman ultraRight;
UltraKalman ultraLeft;
UltraKalman ultraFront;
UltraKalman ultraBack;

///////////////////////////////////////////SHARP KALMAN FILTER///////////////////////////////////////////
struct SharpKalman{
  SharpKalman(){
    varSensor = 3e-6; //Variance of sensor. The LESS, the MORE it looks like the raw input.
    varProcess = 1e-7; 
    P = 1.0;
    Pc = 0.0;
    G = 0.0;
    Xp = 0.0;
    Zp = 0.0;
    Xe = 0.0;
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
  float Xe;
  double rawDistance;
  double distance;
  bool side;
};
SharpKalman sharpRight;
SharpKalman sharpLeft;
SharpKalman sharpFront;

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
float xIMUVarSensor = 10e-4; //Variance of sensor. The LESS, the MORE it looks like the raw input
float xIMUVarProcess = 1e-7; // The greater the variance, faster the sensor response
float xIMUP = 1.0;
float xIMUPc = 0.0;
float xIMUG = 0.0;
float xIMUXp = 0.0;
float xIMUZp = 0.0;
float xIMUXe = 0.0; // Xe

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
//// MPU
//  Serial.println("Initializing MPU6050...");
//  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
//  {
//    Serial.println("MPU6050 has been initiated!");
//    delay(500);
//  }
//  mpu.calibrateGyro();
//  mpu.setThreshold(3);
//// BNO
//////////////////////////////////////////////////////////////////////////////////////////////////
  calibrarColores(1);
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
//// LCD
//  lcd.backlight();
  lcd.begin();
  delay(500);
}

void loop(){
//   filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
//   if(ultraNegativoSide){
//       spinPID(bno, event, mpu, -90, false);
//       stop(false);
//       filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);      
//   }
//  currentColor();

//  rightPriotity(ultraFront, ultraRight, ultraLeft); 
  firstChallenge();
//  currentColor();

//  forwardPID(bno, event, mpu);
//  Serial.println(rawInput);

//  filtrateDistances(ultraFront, ultraRight, ultraLeft);
//  oneStep(ultraFront, ultraRight, ultraLeft, 35);
//  spinPID(bno, event, mpu, 90);
//backPID(bno, event, mpu);

//go();

//  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
//  Serial.print(ultraLeft.distance);
//  Serial.print("\t");
//  Serial.print(ultraFront.distance);
//  Serial.print("\t");
//  Serial.print(ultraRight.distance);
//  Serial.print("\t");
//  Serial.println(ultraBack.distance);

//  oneStepMillis(true);
//  stop(true);
//  delay(2000);

//  calibrarColores(0);
//  while(1){
//    readColor(r, g, b);
//    Serial.print(r);
//    Serial.print("\t");
//    Serial.print(g);
//    Serial.print("\t");
//    Serial.println(b); 
//  } 

//  xBNO_RawKalman(bno, event);
//  ultra_RawKalman(ultraFront, pingFront);
//  ultra_RawKalman(ultraLeft, pingLeft);
//  ultra_RawKalman(ultraRight, pingRight);
//  ultra_RawKalman(ultraBack, pingBack);
}
