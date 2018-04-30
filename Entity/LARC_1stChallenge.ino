void firstControlChallenge() {
  range = 45;
  calibrarColores(1);
  delay(5000);
  while (1) {
    while (currentColor() != 1) {
      forwardPID();
    }
    stop(false);
    backPID(160);
    stop(false);
    spinPID(-90, true);
  }
}

