void firstChallenge(){
  delay(5000);
  while(1){
    while(currentColor() != 1){
      forwardPID(bno, event, mpu);
    }
    stop(false);
//    setNewFakeSetpoint();     
    backPID(bno, event, mpu, 260);
    spinPID(bno, event, mpu, -90, true);  
  }
}

