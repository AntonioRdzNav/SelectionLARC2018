double slowGo(double time){
  return 65*(1-exp(-0.01*time/5));
}

void stop(bool isSpin){   
  analogWrite(motorR1, 0);
  analogWrite(motorR2, 0);
  analogWrite(motorL1, 0);
  analogWrite(motorL2, 0);  
  if(isSpin){    
    analogWrite(motorR1, 235);
    analogWrite(motorR2, 0);
    analogWrite(motorL1, 235);
    analogWrite(motorL2, 0); 
    delay(70);   
  }                                                                  
  analogWrite(motorR1, 0);
  analogWrite(motorR2, 0);
  analogWrite(motorL1, 0);
  analogWrite(motorL2, 0); 
//  delay(100); 
}      

void go() {
  analogWrite(motorR1, 0);
  analogWrite(motorR2, 0);
  analogWrite(motorL1, 0);
  analogWrite(motorL2, 100); 
}

void back() {
  analogWrite(motorR1, 110);
  analogWrite(motorR2, 0);
  analogWrite(motorL1, 110);
  analogWrite(motorL2, 0); 
    delay(2000);
}
//
//void turnRight(){                                                                              
//    analogWrite(motorR1, 10);
//    analogWrite(motorR2, 0);
//    analogWrite(motorL1, 0);
//    analogWrite(motorL2, 10);
//    delay(200);
//    stop(false);
//}
//
//void turnLeft(){                                                                              
//    analogWrite(motorR1, 0);
//    analogWrite(motorR2, 50);
//    analogWrite(motorL1, 50);
//    analogWrite(motorL2, 0);
//    delay(200);
//    stop(false);
//}

