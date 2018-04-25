void regulateOutputsWallPID(){
  if(leftWheelOutput+velGenIzqWall > maxWallVel)
    leftWheelOutput=maxWallVel-velGenIzqWall;
  if(rightWheelOutput+velGenDerWall > maxWallVel)
    rightWheelOutput=maxWallVel-velGenDerWall;
}

void wallDistancesCompute(){
  if(sharpLeft.kalmanDistance < SetpointWallDistance){
    leftWallError = SetpointWallDistance - sharpLeft.kalmanDistance;
    leftWheelOutput = sharpConsKp*leftWallError;
    rightWheelOutput = 0;
  }
  else if(sharpLeft.kalmanDistance > SetpointWallDistance){
    leftWallError = sharpLeft.kalmanDistance - SetpointWallDistance;
    rightWheelOutput = sharpConsKp*leftWallError;
    leftWheelOutput = 0;    
  }
  else{
    leftWheelOutput = 0;
    rightWheelOutput = 0;
  }
}

void leftWallDistancePID() {
  readBNO();    
  filtrateDistancesSharp();    
  wallDistancesCompute();
  regulateOutputsFrontPID();
  if(rightWheelOutput > 10){
    analogWrite(motorR1, 0);
    analogWrite(motorR2, velGenDerWall + rightWheelOutput);
    analogWrite(motorL1, 0);
    analogWrite(motorL2, velGenIzqWall*0.8);  
  }  
  else if(leftWheelOutput > 10){
    analogWrite(motorR1, 0);
    analogWrite(motorR2, velGenDerWall*0.8);
    analogWrite(motorL1, 0);
    analogWrite(motorL2, velGenIzqWall + leftWheelOutput);  
  } 
  else{
    analogWrite(motorR1, 0);
    analogWrite(motorR2, velGenDerWall);
    analogWrite(motorL1, 0);
    analogWrite(motorL2, velGenIzqWall);
  }                  
}

void thirdControlChallenge(){
  leftWallDistancePID();
}

