void regulateOutputsWallPID(){
  if(leftWheelOutput+velGenIzqWall > maxWallVel)
    leftWheelOutput=maxWallVel-velGenIzqWall;
  if(rightWheelOutput+velGenDerWall > maxWallVel)
    rightWheelOutput=maxWallVel-velGenDerWall;
}

void wallDistancesCompute(){
  if(sharpLeft.kalmanDistance < SetpointWallDistance-1.2){
    leftWallError = SetpointWallDistance - sharpLeft.kalmanDistance;
    leftWheelOutput = sharpConsKp*leftWallError;
    rightWheelOutput = 0;
  }
  else if(sharpLeft.kalmanDistance > SetpointWallDistance+1.2){
    leftWallError = sharpLeft.kalmanDistance - SetpointWallDistance;
    rightWheelOutput = sharpConsKp*leftWallError;
    leftWheelOutput = 0;    
  }
  else{
    leftWheelOutput = 0;
    rightWheelOutput = 0;
  }
}

void alignSharpFront(bool frontWall){
  stop(false);  
  if(frontWall){
    while(sharpFront.kalmanDistance < 7){
      analogWrite(motorR1, 140);
      analogWrite(motorR2, 0);
      analogWrite(motorL1, 0);
      analogWrite(motorL2, 140);
      filtrateDistancesSharp();  
    }
  }
  else{
    while(sharpFront.kalmanDistance > 5 && sharpLeft.kalmanDistance > 14){
      analogWrite(motorR1, 0);
      analogWrite(motorR2, 220);
      analogWrite(motorL1, velGenDerWall);
      analogWrite(motorL2, 0);
      filtrateDistancesSharp();  
    }    
  }
}

//bool checkSetpoint(){
//  if((Input*Setpoint < 0) || 
//}

void wallDistancePID() {   
  filtrateDistancesSharp();     
//  Serial.print(sharpLeft.distance);
//  Serial.print("\t");
//  Serial.print(sharpFront.distance);
//  Serial.print("\t");
//  Serial.println(sharpRight.distance);   
  wallDistancesCompute();
  regulateOutputsWallPID();
  Serial.print(leftWheelOutput);
  Serial.print("\t");
  Serial.println(rightWheelOutput);    
  if(sharpFront.kalmanDistance < 7)
    alignSharpFront(true); 
  else if(rightWheelOutput > 0){
    analogWrite(motorL2, velGenDerWall);     
    analogWrite(motorR1, 0);
    analogWrite(motorR2, velGenDerWall + rightWheelOutput);
    analogWrite(motorL1, 0);
  }  
  else if(leftWheelOutput > 0){
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
  velGenDer = 70;
  velGenIzq = 70;  
  leftConsKp=6, leftConsKi=0, leftConsKd=0;
  rightConsKp=6, rightConsKi=0, rightConsKd=0;  
  wallDistancePID();
}

