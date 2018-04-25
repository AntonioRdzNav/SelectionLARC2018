void oneStepMillis(bool comeFromBack){
  double time;
  int tempColor;
  switchColor = false;
  (comeFromBack)? (time = timeStepBack): (time = timeStep);
//  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
  double startTime = millis();
  double endTime = 0;
  while ((endTime - startTime < time)/* && (ultraFront.distance > 8)*/){ 
    updateColors(currentColor());
    if(!switchColor){
      tempColor = currentColor();
      switchColor = (tempColor!=-1) && (tempColor!=4) && (tempColor!=3);
    }
    if(switchColor && currentColor()==4){
      stopColor(tempColor);
      time+=2000;
      switchColor = false; 
    }
    forwardPID();
    endTime = millis();    
//    filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
  } 
}


//void rightPriotity(UltraKalman &ultraFront, UltraKalman &ultraRight, UltraKalman &ultraLeft){ 
//  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
//  if(colorRedDetected && colorGreenDetected && colorBlackDetected){
////    exitRoutine();  
//    while(1){
//      stop(false);
//    }
//  }
//  if((fakeSetpoint*Setpoint)<0 || (abs(fakeSetpoint)-abs(Setpoint))>50)
//    edoTensei();
//  readPosition(bno, event, mpu, 'B');
//  if(!ultraRight.side){
//    if(turnsCounter < 4){ 
//      spinPID(bno, event, mpu, 90, false);
//      turnsCounter++;
//    }
//    else{
//      if(!ultraLeft.side){
//        spinPID(bno, event, mpu, -90, false);  
//        turnsCounter = 0;
//        firstBack=false;
//      }
//      else if(!ultraFront.side){
//        oneStepMillis(firstBack);
//        firstBack=false;
//        turnsCounter = 0;
//      }
//      else{
//        spinPID(bno, event, mpu, 90, false);
//        turnsCounter++;
//      }
//    }
//  }
//  else if (!ultraFront.side){
//    oneStepMillis(firstBack);
//    firstBack=false;
//    turnsCounter = 0;
//  }
//  else if(!ultraLeft.side){
//    spinPID(bno, event, mpu, -90, false);  
//    turnsCounter = 0;
//  }
//  else{
//    filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
//    turnsCounter = 0;
//    if(ultraRight.distance > ultraLeft.distance)
//      spinPID(bno, event, mpu, 180, false);
//    else{
//      oneStepMillis(firstBack);
//      spinPID(bno, event, mpu, -180, false);
//    }
//  }  
//}
