void rawUltrasonic(UltraKalman &ultra, NewPing ultraPing){
  double ultraTemp = ultraPing.ping_cm();
  if(ultraTemp<200 && ultraTemp!=0){
    if(ultraTemp > 95)
      ultra.distance = 95;
    else
      ultra.distance = ultraTemp;
  }
}
double filterRawSharp(double rawSharpMeasure){
  if(rawSharpMeasure<200 && rawSharpMeasure!=0){
    if(rawSharpMeasure > 95)
      return 95;
    else
      return rawSharpMeasure;
  }  
}

void calculateRawDistances(UltraKalman &ultraFront, UltraKalman &ultraRight, UltraKalman &ultraLeft){
  rawUltrasonic(ultraRight, pingRight);
  rawUltrasonic(ultraLeft, pingLeft);
  rawUltrasonic(ultraFront, pingFront);
}
// There are different sharps:
//  -ShortDistance Sharp = 2076 / (sensorvalue-11)
//  -MediumDistance Sharp = 4800 / (sensorvalue-20)
//  -LongDistance Sharp = 9462 / (sensorvalue-16.92)
void calculateRawDistancesSharp(){
  sharpRight.rawDistance = analogRead(sharpRightPin);
  sharpRight.distance = filterRawSharp(sharpRight.rawDistance);
  sharpFront.rawDistance = analogRead(sharpLeftPin);
  sharpFront.distance = filterRawSharp(sharpFront.rawDistance);
  sharpLeft.rawDistance = analogRead(sharpFrontPin);
  sharpLeft.distance = filterRawSharp(sharpLeft.rawDistance);
}
