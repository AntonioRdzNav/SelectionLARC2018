void firstControlChallenge() {
  range = 42;
  rightTurnKp= 11;
  leftTurnKp=11;
  calibrarColores(1);
  delay(5000);
  while (1) {
    while (currentColor() != 1) {
      forwardPID();
    }
    stop(false);
    backPID(225);
    stop(false);
    spinPID(-90, true);
  }
}

