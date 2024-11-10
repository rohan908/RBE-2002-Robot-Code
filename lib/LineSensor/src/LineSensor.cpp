#include "LineSensor.h"

void LineSensor::Initialize(void)
{
    /*qtr.setTypeAnalog();
    qtr.setSensorPins(lineSensorPins, numSensors);
    qtr.setSamplesPerSensor(numSamplesPerSensor);
    qtr.setEmitterPin(emitterPin);
    qtr.setNonDimmable();
    */
   for (uint8_t i = 0; i <  numSensors; i++){
    calibrateMins[i] = 2000;
    calibrateMaxs[i] = 0;
   }
}

void LineSensor::read(){
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    sensorValues[i] = 0;
  }

  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    sensorValues[i] += analogRead(lineSensorPins[i]);
  }

  //Gotten from QTR Library
  // get the rounded average of the readings for each sensor
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    sensorValues[i] = (sensorValues[i] + (numSamplesPerSensor >> 1)) /
      numSamplesPerSensor;
  }
}


float LineSensor::readLine(void){
  uint32_t avg= 0;
  uint16_t sum = 0;
  readCalibrated();
  uint16_t value;
  for (uint8_t i = 0; i < numSensors; i++)
  {
    value = isLineBlack ? 1000 - sensorValues[i] : sensorValues[i]; //flips the values if its sensing black lines
    //Modified from QTR Polulu Library
    if (value > 200) {onLine = true;
      //Serial.println("On Line!");
    }

    // only average in values that are above a noise threshold
    // Weighted average
    if (value > 50)
    {
      avg += (uint32_t)value * (i * 1000);
      sum += value;
    }
  }

  if (!onLine)
  {
    //Serial.println("Not On Line!");
    // If it last read to the left of center, return 0.
    if (lastPosition < (numSensors - 1) * 1000 / 2)
    {
      return 0;
    }
    // If it last read to the right of center, return the max (5000).
    else
    {
      return (numSensors - 1) * 1000;
    }
  }

  // results in a position between 0 to 5000 where 0 is the farthest left sensor and 5000 is the farthest right sensor
  lastPosition = avg / sum;
  /*if (calibrated){
    lastPosition = constrain((lastPosition - 1250) * 2, 0, 5000); //line for mapping to wider range (necessary after line sensor change bruh )
  }*/
  #ifdef __LINE_FOLLOW_DEBUG__
    Serial.print(">position:");
    Serial.println(lastPosition);
  #endif
  return lastPosition;
}

void LineSensor::Calibrate(){
  read();
  for (uint8_t i = 0; i < numSensors; i++){
    if (calibrateMaxs[i] < sensorValues[i]){
      calibrateMaxs[i] = sensorValues[i];
    }
    if(calibrateMins[i] > sensorValues[i]){
      calibrateMins[i] = sensorValues[i];
    }
  }
  calibrated = true;
  #ifdef __LINE_FOLLOW_DEBUG__
  Serial.println("Calibrating!");
  /*for (int i = 0; i < NUM_SENSORS; i++){
      Serial.print(calibrateMins[i] + "\t" + calibrateMaxs[i]);
      Serial.println();
  }
  */
  Serial.println();
  #endif
}

void LineSensor::readCalibrated()
{
  // read the needed values
  read();
  if (calibrated){

    for (uint8_t i = 0; i < numSensors; i++)
    {
      uint16_t calmin, calmax;
      calmax = calibrateMaxs[i];
      calmin = calibrateMins[i];

      uint16_t denominator = calmax - calmin;
      int16_t value = 0;

      if (denominator != 0)
      {
        value = (((int32_t)sensorValues[i]) - calmin) * 1000 / denominator;
      }

      if (value < 0) { value = 0; }
      else if (value > 1000) { value = 1000; }

      sensorValues[i] = value;
    }
  }
}




float LineSensor::CalcError(void) 
{
    float position = readLine();
    if (position == 5000){ //went off the line
      return 5;
    }
    if (position == 0){//went off the line
      return -5;
    }
    return (position - 2500) / 1000;

}
    

bool LineSensor::CheckIntersection(void)
{
  bool retVal = false;
  readCalibrated();
  bool isLeftDark;
  bool isRightDark;
  if (isLineBlack){
    isLeftDark = sensorValues[0] > DARK_THRESHOLD;
    isRightDark = sensorValues[numSensors - 1] > DARK_THRESHOLD;
  }
  else{
    isLeftDark = sensorValues[0] < WHITE_THRESHOLD;
    isRightDark = sensorValues[numSensors - 1] < WHITE_THRESHOLD;
  }
  

  bool onIntersection = isLeftDark && isRightDark;
  if(onIntersection && !prevOnIntersection) retVal = true;

  prevOnIntersection = onIntersection;


  return retVal;
    
}