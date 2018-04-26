void regulateOutputsWallPID(){
  if(leftWheelOutput+velGenIzqWall > maxWallVel)
    leftWheelOutput=maxWallVel-velGenIzqWall;
  if(rightWheelOutput+velGenDerWall > maxWallVel)
    rightWheelOutput=maxWallVel-velGenDerWall;
}

void wallDistancesCompute(){
  if(sharpLeft.kalmanDistance < SetpointWallDistance-1.5){
    leftWallError = SetpointWallDistance - sharpLeft.kalmanDistance;
    leftWheelOutput = sharpConsKp*leftWallError;
    rightWheelOutput = 0;
  }
  else if(sharpLeft.kalmanDistance > SetpointWallDistance+1.5){
    leftWallError = sharpLeft.kalmanDistance - SetpointWallDistance;
    rightWheelOutput = sharpConsKp*leftWallError;
    leftWheelOutput = 0;    
  }
  else{
    leftWheelOutput = 0;
    rightWheelOutput = 0;
  }
}

void alignSharpFront(){
  while(sharpFront.kalmanDistance < 11){
    analogWrite(motorR1, 160);
    analogWrite(motorR2, 0);
    analogWrite(motorL1, 0);
    analogWrite(motorL2, 160);
    filtrateDistancesSharp();  
  }
}

//bool checkSetpoint(){
//  if((Input*Setpoint < 0) || 
//}

void wallDistancePID() {   
  filtrateDistancesSharp();    
  wallDistancesCompute();
  regulateOutputsWallPID();
  if(sharpFront.kalmanDistance < 11)
    alignSharpFront(); 
  else if(rightWheelOutput > 15){
    analogWrite(motorR1, 0);
    analogWrite(motorR2, velGenDerWall + rightWheelOutput);
    analogWrite(motorL1, 0);
    analogWrite(motorL2, velGenIzqWall);  
  }  
  else if(leftWheelOutput > 15){
    analogWrite(motorR1, 0);
    analogWrite(motorR2, velGenDerWall);
    analogWrite(motorL1, 0);
    analogWrite(motorL2, velGenIzqWall + leftWheelOutput);  
  } 
  else{
    Setpoint = Input;
    forwardPID(); 
  }                 
}

void thirdControlChallenge(){
  velGenDer = velGenDerWall;
  velGenIzq = velGenIzqWall;  
  wallDistancePID();
}

