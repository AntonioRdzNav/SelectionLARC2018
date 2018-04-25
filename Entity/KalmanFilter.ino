/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////ULTRASONIC/////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
//void ultraKalmanFilter(UltraKalman &ultra){
//	ultra.Pc = ultra.P + ultra.varProcess;
//	ultra.G = ultra.Pc/(ultra.Pc + ultra.varSensor); 	//Kalman gain
//	ultra.P = (1-ultra.G)*ultra.Pc;
//	ultra.Xp = ultra.kalmanDistance;
//	ultra.Zp = ultra.Xp;
//	ultra.kalmanDistance = ultra.G*(ultra.distance-ultra.Zp)+ultra.Xp;	//Estimates new filtered input
//}
//void ultra_RawKalman(UltraKalman &ultra, NewPing ultraPing){
//  rawUltrasonic(ultra, ultraPing);
//  Serial.print(ultra.distance);
//  Serial.print(",");
//  ultraKalmanFilter(ultra);
//  Serial.println(ultra.kalmanDistance);
//}
//void filtrateDistances(UltraKalman &ultraFront, UltraKalman &ultraRight, UltraKalman &ultraLeft, UltraKalman &ultraBack){
//  calculateRawDistances(ultraFront, ultraRight, ultraLeft);
//  ultraKalmanFilter(ultraFront);
//  ultraKalmanFilter(ultraRight);
//  ultraKalmanFilter(ultraLeft);
//  ultraKalmanFilter(ultraBack);
//  if(ultraFront.kalmanDistance <= 12 && ultraFront.kalmanDistance!=0) ultraNegativoSide = true;
//  else ultraNegativoSide = false;
//  if(ultraBack.kalmanDistance <= 25 && ultraBack.kalmanDistance!=0) ultraBack.side = true;
//  else ultraBack.side = false;
//  if(ultraFront.kalmanDistance <= 25 && ultraFront.kalmanDistance!=0) ultraFront.side = true;
//  else ultraFront.side = false;
//  if(ultraRight.kalmanDistance <= 25 && ultraRight.kalmanDistance != 0) ultraRight.side = true;
//  else ultraRight.side = false;
//  if(ultraLeft.kalmanDistance <= 25 && ultraLeft.kalmanDistance != 0) ultraLeft.side = true;
//  else ultraLeft.side = false;
//}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////SHARP///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void sharpKalmanFilter(SharpKalman &sharp){
  sharp.Pc = sharp.P + sharp.varProcess;
  sharp.G = sharp.Pc/(sharp.Pc + sharp.varSensor);  //Kalman gain
  sharp.P = (1-sharp.G)*sharp.Pc;
  sharp.Xp = sharp.kalmanDistance;
  sharp.Zp = sharp.Xp;
  sharp.kalmanDistance = sharp.G*(sharp.distance-sharp.Zp)+sharp.Xp;  //Estimates new filtered input
}
void sharp_RawKalman(SharpKalman &sharp){
  calculateRawDistancesSharp();
  Serial.print(sharp.distance);
  Serial.print(",");
  sharpKalmanFilter(sharp);
  Serial.println(sharp.kalmanDistance);
}
void filtrateDistancesSharp(){
  calculateRawDistancesSharp();
  sharpKalmanFilter(sharpFront);
  sharpKalmanFilter(sharpRight);
  sharpKalmanFilter(sharpLeft);
  if(sharpFront.kalmanDistance <= 25 && sharpFront.kalmanDistance!=0) sharpFront.side = true;
  else sharpFront.side = false;
  if(sharpRight.kalmanDistance <= 25 && sharpRight.kalmanDistance != 0) sharpRight.side = true;
  else sharpRight.side = false;
  if(sharpLeft.kalmanDistance <= 25 && sharpLeft.kalmanDistance != 0) sharpLeft.side = true;
  else sharpLeft.side = false;    
}

/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////BNO////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void xBNOKalmanFilter(){
  xBNOPc = xBNOP + xBNOVarProcess;
  xBNOG = xBNOPc/(xBNOPc + xBNOVarSensor);  //Kalman gain
  xBNOP = (1-xBNOG)*xBNOPc;
  xBNOXp = xBNOXe;
  xBNOZp = xBNOXp;
  xBNOXe = xBNOG*(Input-xBNOZp)+xBNOXp;  //Estimates new filtered input
}
void xBNO_RawKalman(Adafruit_BNO055 &bno, sensors_event_t &event){
  readBNO();
  Serial.print(Input);
  Serial.print(",");
  xBNOKalmanFilter();
  Serial.println(xBNOXe);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////IMU////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
//void xIMUKalmanFilter(){
//  xIMUPc = xIMUP + xIMUVarProcess;
//  xIMUG = xIMUPc/(xIMUPc + xIMUVarSensor);  //Kalman gain
//  xIMUP = (1-xIMUG)*xIMUPc;
//  xIMUXp = xIMUXe;
//  xIMUZp = xIMUXp;
//  xIMUXe = xIMUG*(Input-xIMUZp)+xIMUXp;  //Estimates new filtered input
//}
//void xIMU_RawKalman(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu){
//  readPosition(bno, event, mpu, 'I');
//  Serial.print(Input);
//  Serial.print(",");
//  xIMUKalmanFilter();
//  Serial.println(xIMUXe);
//}
