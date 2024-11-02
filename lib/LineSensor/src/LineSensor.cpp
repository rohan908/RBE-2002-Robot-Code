#include "LineSensor.h"

#define DARK_THRESHOLD 500;

void LineSensor::Initialize(void)
{
    /*qtr.setTypeAnalog();
    qtr.setSensorPins(lineSensorPins, numSensors);
    qtr.setSamplesPerSensor(numSamplesPerSensor);
    qtr.setEmitterPin(emitterPin);
    qtr.setNonDimmable();
    */
   
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


float LineSensor::readLineBlack(){
  uint32_t avg= 0;
  uint16_t sum = 0;
  read();
  for (uint8_t i = 0; i < numSensors; i++)
  {
    uint16_t value = 1000 - sensorValues[i]; //flips the values so black lines are what is good
    
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
    Serial.println("Not On Line!");
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
  return lastPosition;
}


float LineSensor::CalcError(void) 
{
    float position = readLineBlack();
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
    return false;
    /*
    bool retVal = false;

    
    bool isLeftDark = analogRead(leftSensorPin) > DARK_THRESHOLD;
    bool isRightDark = analogRead(rightSensorPin) > DARK_THRESHOLD;

    bool onIntersection = isLeftDark && isRightDark;
    if(onIntersection && !prevOnIntersection) retVal = true;

    prevOnIntersection = onIntersection;

    return retVal;
    */
}