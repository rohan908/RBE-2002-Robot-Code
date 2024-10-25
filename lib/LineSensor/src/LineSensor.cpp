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

      // get the rounded average of the readings for each sensor
      for (uint8_t i = 0; i < 6; i++)
      {
        sensorValues[i] = (sensorValues[i] + (numSamplesPerSensor >> 1)) /
          numSamplesPerSensor;
      }
}

void LineSensor::Calibrate(){
  uint16_t maxSensorValues[numSensors];
  uint16_t minSensorValues[numSensors];

  // (Re)allocate and initialize the arrays if necessary.
  // Memory allocation code from POLOLU code base
  if (!calibration.initialized)
  {
    uint16_t * oldMaximum = calibration.maximum;
    calibration.maximum = (uint16_t *)realloc(calibration.maximum,
                                              sizeof(uint16_t) * numSensors);
    if (calibration.maximum == nullptr)
    {
      // Memory allocation failed; don't continue.
      free(oldMaximum); // deallocate any memory used by old array
      return;
    }

    uint16_t * oldMinimum = calibration.minimum;
    calibration.minimum = (uint16_t *)realloc(calibration.minimum,
                                              sizeof(uint16_t) * numSensors);
    if (calibration.minimum == nullptr)
    {
      // Memory allocation failed; don't continue.
      free(oldMinimum); // deallocate any memory used by old array
      return;
    }

    // Initialize the max and min calibrated values to values that
    // will cause the first reading to update them.
    for (uint8_t i = 0; i < numSensors; i++)
    {
      calibration.maximum[i] = 0;
      calibration.minimum[i] = 9000;
    }

    calibration.initialized = true;
  }

  for (uint8_t j = 0; j < 10; j++)
  {
    read();

    for (uint8_t i = 0; i < numSensors; i++)
    {
      // set the max we found THIS time
      if ((j == 0) || (sensorValues[i] > maxSensorValues[i]))
      {
        maxSensorValues[i] = sensorValues[i];
      }

      // set the min we found THIS time
      if ((j == 0) || (sensorValues[i] < minSensorValues[i]))
      {
        minSensorValues[i] = sensorValues[i];
      }
    }
  }

  // record the min and max calibration values
  for (uint8_t i = 0; i < numSensors; i++)
  {
    // Update maximum only if the min of 10 readings was still higher than it
    // (we got 10 readings in a row higher than the existing maximum).
    if (minSensorValues[i] > calibration.maximum[i])
    {
      calibration.maximum[i] = minSensorValues[i];
    }

    // Update minimum only if the max of 10 readings was still lower than it
    // (we got 10 readings in a row lower than the existing minimum).
    if (maxSensorValues[i] < calibration.minimum[i])
    {
      calibration.minimum[i] = maxSensorValues[i];
    }
  }
}

void LineSensor::readCalibration(){
    for (int  i = 0; i < numSensors; i++){
        
        uint16_t denominator = calibration.maximum[i] - calibration.minimum[i];
        int16_t value = 0;

        if (denominator != 0)
        {
            value = (((int32_t)sensorValues[i]) - calibration.minimum[i]) * 1000 / denominator;
        }

        if (value < 0) { value = 0; }
        else if (value > 1000) { value = 1000; }

        sensorValues[i] = value;
    }

}

float LineSensor::readLineBlack(){
    uint32_t avg= 0;
    uint16_t sum = 0;
    for (uint8_t i = 0; i < numSensors; i++)
    {
    uint16_t value = 1000 - sensorValues[i];
    

    // keep track of whether we see the line at all
    if (value > 200) {onLine = true;
      Serial.println("On Line!");
    }

    // only average in values that are above a noise threshold
    if (value > 50)
    {
      avg += (uint32_t)value * (i * 1000);
      sum += value;
    }
  }

  if (!onLine)
  {
    // If it last read to the left of center, return 0.
    if (lastPosition < (numSensors - 1) * 1000 / 2)
    {
      return 0;
    }
    // If it last read to the right of center, return the max.
    else
    {
      return (numSensors - 1) * 1000;
    }
  }

  lastPosition = avg / sum;
  return lastPosition;
}


float LineSensor::CalcError(void) 
{
    float position = readLineBlack();
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