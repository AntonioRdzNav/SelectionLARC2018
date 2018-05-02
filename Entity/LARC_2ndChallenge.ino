void readInitialColors(bool willReadColors){
  int tempColor, lastColor=-1;
  int nInitialColors=0;
  switchColor = false;  
  while(nInitialColors<3){ 
    if(!switchColor){  
      tempColor = currentColor(); 
      switchColor = (tempColor!=-1) && ((tempColor==0) || (tempColor==1) || (tempColor==2)) && (tempColor!=lastColor);    
    }
    if(switchColor && currentColor()!=tempColor){
      if(willReadColors)
        stopColor(tempColor);
      switchColor = false; 
      if(willReadColors && tempColor!=4){//Will stack the colors and the color is not white
        initialColors[nInitialColors] = tempColor;  
      }
      goBox(100);      
      nInitialColors++;        
    }
    if(tempColor != -1)
      lastColor = tempColor;
    forwardPID();    
  }
}

void goUntilBlack(int blackTimes){
  int tempColor;
  int blackDetected=0;
  switchColor = false;  
  while(blackDetected<blackTimes){ 
    if(!switchColor){  
      tempColor = currentColor(); 
      switchColor = (tempColor!=-1) && (tempColor==3);    
    }
    if(switchColor && currentColor()==4){
      switchColor = false; 
      blackDetected++;     
    }
    forwardPID();
  }   
  stop(false);  
}

void goUntilBox(bool isRightSharp){ 
  filtrateDistancesSharp();
  if(isRightSharp){
    while(!sharpRight.side){ 
      forwardPID();
      filtrateDistancesSharp();
    }   
  }
  else{
    while(!sharpLeft.side){ 
      forwardPID();
      filtrateDistancesSharp();
    } 
  }
  stop(false);  
}

void goUntilGreen(){
  int tempColor;
  bool greenDetected=false;
  switchColor = false;  
  while(!greenDetected){ 
    if(!switchColor){  
      tempColor = currentColor(); 
      switchColor = (tempColor!=-1) && (tempColor==2);    
    }
    if(switchColor && currentColor()==4){
      switchColor = false; 
      greenDetected=true;     
    }
    forwardPID();
  }   
  stop(false);     
}

void goBox(double time){
  double startTime = millis();
  while (millis() < startTime + time){ 
    forwardPID();
  }   
  stop(false);
}

void goRedBox(){
  filtrateDistancesSharp();
  goBox(700); 
  spinPID(-90, true);
  goBox(1100);
  goUntilBox(true);
//  goBox(400);
  stop(false);
  turnOnRedLED();
  spinPID(-90, true); 
}

void goGreenBox(){
  goBox(700);  
  spinPID(-90, true); 
  stop(false);
  turnOnGreenLED();
  spinPID(-90, true);
}

void goBlueBox(){
  filtrateDistancesSharp();
  goBox(700); 
  spinPID(90, true);
  goBox(1100);
  goUntilBox(false);
//  goBox(400);  
  spinPID(90, true);
  spinPID(90, true);
  stop(false);
  turnOnBlueLED();
}

void returnBegin(char color){ 
  if(color == 'R'){
    filtrateDistancesSharp();
//    goUntilGreen();  
    spinPID(-90, true);
    goBox(1100);
    goUntilBox(false);
    backPID(300);
    spinPID(90, true);
    goUntilBlack(1);
    backPID(400);  
    spinPID(180, true);
//    goBox(400);
//    goBox(4200);
  }
  else if(color == 'B'){
    filtrateDistancesSharp();
//    goUntilGreen(); 
    goBox(1100);
    goUntilBox(true);
    backPID(300);  
    spinPID(-90, true);
    goUntilBlack(1);
    backPID(400);  
    spinPID(-180, true);
//    goBox(400);    
//    goBox(4200);
  }
  else if(color == 'G'){
//    spinPID(-90, true);
//    goUntilGreen(); 
//    spinPID(90, true);
//    goBox(1300);
//    spinPID(-90, true);
    goUntilBlack(1);
    backPID(400);  
    spinPID(180, true);
//    goBox(400);
//    goBox(5000);
  }
//  spinPID(-180, true);
}

void secondControlChallenge(){
  range = 23;
  rightTurnKp= 11;
  leftTurnKp=11;
  calibrarColores(0);
  readInitialColors(true);  
  for(int i=0; i<3; i++){
    goUntilGreen();
    if(i>0){
      goBox(950); 
      goUntilGreen();
    }
    if(initialColors[i] == 0){
      goRedBox();
      returnBegin('R');
    }
    else if(initialColors[i] == 1){
      goBlueBox();
      returnBegin('B');
    }
    else if(initialColors[i] == 2){
      goGreenBox();
      returnBegin('G');
    }
  }
  while(1){
    stop(false);
  }
}

