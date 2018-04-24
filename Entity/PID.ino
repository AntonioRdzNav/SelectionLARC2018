// -90 to turn left
// 0 to go forward
// 90 to turn right
void calculateNewSetpoint(int angle) {
  calculateFakeSetpoint(angle);
  int newSetpoint = Setpoint;
  newSetpoint += angle;
  if (newSetpoint < -180)
    newSetpoint += 360;
  if (newSetpoint > 180)
    newSetpoint -= 360;
  Setpoint = newSetpoint;
}
void calculateFakeSetpoint(int angle) {
  if(spinRight)
    angle-=offsetAngle;
  if(spinLeft)
    angle+=offsetAngle;
  int newFakeSetpoint = fakeSetpoint;
  newFakeSetpoint += angle;
  if (newFakeSetpoint < -180)
    newFakeSetpoint += 360;
  if (newFakeSetpoint > 180)
    newFakeSetpoint -= 360;
  fakeSetpoint = newFakeSetpoint;
}
void setNewFakeSetpoint(){
  fakeSetpoint = Input;
}

void edoTensei(){
  fakeSetpoint = Setpoint;
}

void regulateOutputsTurnPID(){
  if(leftOutput > maxTurnVel)
    leftOutput=maxTurnVel ;
  if(rightOutput > maxTurnVel)
    rightOutput=maxTurnVel;
}

void regulateOutputsFrontPID(){
  if(leftOutput+velGenDer > maxFrontVel)
    leftOutput=maxFrontVel-velGenDer;
  if(rightOutput+velGenIzq > maxFrontVel)
    rightOutput=maxFrontVel-velGenIzq;
}

void backPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu, int time) {
  leftPID.SetTunings(rightConsKp-1.2, rightTurnKi, rightTurnKd);
  rightPID.SetTunings(leftConsKp-1.2, leftTurnKi, leftTurnKd);
  double startTime = millis();
  double endTime = 0;
  setFakeSetpoint=false;
  readPosition(bno, event, mpu, 'B');
  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack); 
  while (endTime - startTime < time)/* && (!limit1 || !limit2))*/ {  
    leftPID.Compute();  //Gets an output
    rightPID.Compute();
    regulateOutputsFrontPID();   
    if(abs(Setpoint)==180){
      if(Setpoint==180){
        if (Input>0) {
          analogWrite(motorR1, velGenDerBack + leftOutput);
          analogWrite(motorR2, 0);
          analogWrite(motorL1, velGenIzqBack);
          analogWrite(motorL2, 0);                     
        }
        else { 
          analogWrite(motorR1, velGenDerBack);
          analogWrite(motorR2, 0);
          analogWrite(motorL1, velGenIzqBack + leftOutput);
          analogWrite(motorL2, 0);                   
        }
      }        
      else if(Setpoint==-180){
        if (Input>0) {
          analogWrite(motorR1, velGenDerBack + rightOutput);
          analogWrite(motorR2, 0);
          analogWrite(motorL1, velGenIzqBack);
          analogWrite(motorL2, 0);                  
        }
        else {
          analogWrite(motorR1, velGenDerBack);
          analogWrite(motorR2, 0);
          analogWrite(motorL1, velGenIzqBack + rightOutput);
          analogWrite(motorL2, 0);                 
        }
      }      
    }
    else{
      analogWrite(motorR1, velGenDerBack + leftOutput);
      analogWrite(motorR2, 0);
      analogWrite(motorL1, velGenIzqBack + rightOutput);
      analogWrite(motorL2, 0);        
    }
    readPosition(bno, event, mpu, 'B');
    filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack); 
    setFakeSetpoint=true;
    endTime = millis();
  }  
  leftPID.SetTunings(rightConsKp, rightConsKi, rightConsKd);
  rightPID.SetTunings(leftConsKp, leftConsKi, leftConsKd);
  stop(false);
}

void forwardPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu) {
  readPosition(bno, event, mpu, 'B');     
  leftPID.Compute();  //Gets an output
  rightPID.Compute();
  regulateOutputsFrontPID();
  //Aqui empieza
  if(abs(Setpoint) == 180){
    if(fakeSetpoint > 0){
      if(Input > 0 && Input < fakeSetpoint){
        analogWrite(motorR1, 0);
        analogWrite(motorR2, velGenDer);
        analogWrite(motorL1, 0);
        analogWrite(motorL2, velGenIzq + leftOutput + rightOutput);                    
      }
      else{
        analogWrite(motorR1, 0);
        analogWrite(motorR2, velGenDer + (leftOutput + rightOutput)/**(3.5)*/);
        analogWrite(motorL1, 0);
        analogWrite(motorL2, velGenIzq);  
      }      
    }
    else{
      if(Input < 0 && Input > fakeSetpoint){
        analogWrite(motorR1, 0);
        analogWrite(motorR2, velGenDer + (leftOutput + rightOutput)/**(3.5)*/);
        analogWrite(motorL1, 0);
        analogWrite(motorL2, velGenIzq);         
      }
      else{
        analogWrite(motorR1, 0);
        analogWrite(motorR2, velGenDer);
        analogWrite(motorL1, 0);
        analogWrite(motorL2, velGenIzq + leftOutput + rightOutput);   
      }
    }    
  }
  else{
    analogWrite(motorR1, 0);
    analogWrite(motorR2, velGenDer + rightOutput);
    analogWrite(motorL1, 0);
    analogWrite(motorL2, velGenIzq + leftOutput);          
  }
//  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);   
  ledsPID();
}

void spinPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu, int newAngle, bool isDeadEnd){
  spinRight=false, spinLeft=false;  
  (newAngle>0)? spinRight=true: spinLeft=true;
  if(!LARC)
    stop(true);
  bool canBack=false;
  lastSetpoint = Setpoint;
  calculateNewSetpoint(newAngle);
  leftPID.SetTunings(rightTurnKp, rightTurnKi, rightTurnKd);
  rightPID.SetTunings(leftTurnKp, leftTurnKi, leftTurnKd);
  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack); 
  if(newAngle>0){
    if(ultraLeft.side)   
      canBack=true;
  }
  else{
    if(ultraRight.side)
      canBack=true;
  }
  if(LARC)
    canBack=false;
  turnPID(bno, event, mpu, 1500);//1600
  if(abs(fakeSetpoint)-abs(Input) > 50){
    leftPID.SetTunings(rightTurnKp, rightAlignKi, rightAlignKd);
    rightPID.SetTunings(leftTurnKp, leftAlignKi, leftAlignKd);
  }
  else if(abs(fakeSetpoint)-abs(Input)<=50 && abs(fakeSetpoint)-abs(Input)>30){
    double middleLeft = (leftTurnKp+leftAlignKp) / 2.0;
    double middleRight = (rightTurnKp+rightAlignKp) / 2.0;
    leftPID.SetTunings(middleRight, rightAlignKi, rightAlignKd);
    rightPID.SetTunings(middleLeft  , leftAlignKi, leftAlignKd);
  }
  else if(abs(fakeSetpoint)-abs(Input) < 30){
    leftPID.SetTunings(rightAlignKp, rightAlignKi, rightAlignKd);
    rightPID.SetTunings(leftAlignKp, leftAlignKi, leftAlignKd);
  }
  turnPID(bno, event, mpu, 800);//900
  leftPID.SetTunings(rightConsKp, rightConsKi, rightConsKd);
  rightPID.SetTunings(leftConsKp, leftConsKi, leftConsKd);
//  if(canBack && !isDeadEnd){
//     backPID(bno, event, mpu, 1300);     
//     stop(false);
//     if(setFakeSetpoint)
//        setNewFakeSetpoint();   
//     oneStepMillis(true);
//  }
//  else
//    oneStepMillis(false);
}

bool isSetpointReached(){
  if(leftOutput<4 && rightOutput<4)
    return true;
  return false;
}

void turnPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu, double time) {
  double startTime = millis();
  double endTime = 0;
  double tempOutput;
  int setpointReachedCounter=0;
  readPosition(bno, event, mpu, 'B');
//  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack); 
  do{
    bno.getEvent(&event);
    leftPID.Compute();  //Gets an output
    rightPID.Compute();
    regulateOutputsTurnPID();
    if(isSetpointReached())
      setpointReachedCounter++;
    (leftOutput==0)? tempOutput=rightOutput: tempOutput=leftOutput;
    if(Setpoint==180){
      if (Input>0) {
        analogWrite(motorR1, tempOutput);
        analogWrite(motorR2, 0);
        analogWrite(motorL1, 0);
        analogWrite(motorL2, tempOutput);            
      }
      else {
        analogWrite(motorR1, 0);
        analogWrite(motorR2, tempOutput);
        analogWrite(motorL1, tempOutput);
        analogWrite(motorL2, 0);                
      }      
    }
    else if(Setpoint==-180){ //MOVED
      if (Input<0) {
        analogWrite(motorR1, 0);
        analogWrite(motorR2, tempOutput);
        analogWrite(motorL1, tempOutput);
        analogWrite(motorL2, 0);      
      }
      else {
        analogWrite(motorR1, tempOutput);
        analogWrite(motorR2, 0);
        analogWrite(motorL1, 0);
        analogWrite(motorL2, tempOutput);         
      }      
    }
    else{
      if (rightOutput == 0) {
        analogWrite(motorR1, leftOutput);
        analogWrite(motorR2, 0);
        analogWrite(motorL1, 0);
        analogWrite(motorL2, leftOutput);                   
      }
      else {
        analogWrite(motorR1, 0);
        analogWrite(motorR2, rightOutput);
        analogWrite(motorL1, rightOutput);
        analogWrite(motorL2, 0);                    
      }
    }
    ledsPID();    
    readPosition(bno, event, mpu, 'B');
//    filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);    
    endTime = millis();
  }while((endTime-startTime < time) && (setpointReachedCounter<7));
}

