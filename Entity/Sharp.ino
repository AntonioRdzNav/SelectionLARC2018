double filterRawSharp(double rawSharpMeasure){
  if(rawSharpMeasure > 30)
    return 30;
  else
    return rawSharpMeasure;  
}
// There are different sharps:
//  -ShortDistance Sharp = 2076 / (sensorvalue-11)
//  -MediumDistance Sharp = 4800 / (sensorvalue-20)
//  -LongDistance Sharp = 9462 / (sensorvalue-16.92)
void calculateRawDistancesSharp(){
  double volts;
  delay(25); //Right time to make another read
  volts = analogRead(sharpRightPin);
  if(volts>=80 && volts<=530){
    sharpRight.rawDistance = (2076)/(volts-11);
    sharpRight.distance = filterRawSharp(sharpRight.rawDistance);
  }
  volts = analogRead(sharpFrontPin);
  if(volts>=80 && volts<=530){  
    sharpFront.rawDistance = (2076)/(volts-11);
    sharpFront.distance = filterRawSharp(sharpFront.rawDistance);
  }
  volts = analogRead(sharpLeftPin);
  if(volts>=80 && volts<=530){  
    sharpLeft.rawDistance = (2076)/(volts-11);
    sharpLeft.distance = filterRawSharp(sharpLeft.rawDistance);
  }
}
