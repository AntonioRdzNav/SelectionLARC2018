void halfTurn(){
  analogWrite(motorR1, 100);
  analogWrite(motorR2, 0);
  analogWrite(motorL1, 0);
  analogWrite(motorL2, 100); 
  delay(800);
}

void firstControlChallenge() {
  range = 30;
  calibrarColores(1);
  delay(5000);
  while (1) {
    while (currentColor() != 1) {
      forwardPID();
    }
    stop(false);
    backPID(140);
    stop(false);
    spinPID(-90, true);
  }
}

