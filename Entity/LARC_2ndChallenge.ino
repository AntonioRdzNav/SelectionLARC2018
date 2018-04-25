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
  goBox(300);
  spinPID(-90, true);
  goUntilBlack(2);
  spinPID(90, true);
  goBox(1500);
  turnOnRedLED();
}

void goGreenBox(){
  goBox(300);  
  spinPID(-90, true); 
  goUntilBlack(1);
  spinPID(90, true);
  goBox(1500); 
  turnOnGreenLED();  
}

void goBlueBox(){
  goBox(300);  
  spinPID(90, true);
  goUntilBlack(2);
  spinPID(-90, true);
  goBox(1500);
  turnOnBlueLED();
}

void returnBegin(char color){ 
  if(color == 'R'){
    spinPID(-180, true);
    goUntilGreen();  
    spinPID(-90, true);
    goBox(1300);
    spinPID(90, true);
    goBox(5900);
  }
  else if(color == 'B'){
    spinPID(180, true); 
    goUntilGreen(); 
    spinPID(90, true);
    goBox(1300);
    spinPID(-90, true);
    goBox(5900);
  }
  else if(color == 'G'){
    spinPID(-180, true); 
    goUntilGreen();   
    spinPID(-90, true);
    goBox(900);
    spinPID(90, true);
    goBox(5900);
  }
  spinPID(-180, true);
}

void secondControlChallenge(){
  range = 20;
  calibrarColores(0);
  readInitialColors(true);  
  for(int i=0; i<3; i++){
    goUntilBlack(1);
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
}

