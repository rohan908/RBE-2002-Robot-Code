#pragma once

#include <Arduino.h>
#include <QTRSensors.h>

#define LINE_CONTROL_OUT A0
#define LINE_SENSOR_1 A2
#define LINE_SENSOR_3 A4
#define LINE_SENSOR_5 A3
#define LINE_SENSOR_7 A6
#define LINE_SENSOR_9 A1
#define LINE_SENSOR_11 A11 
#define NUM_SENSORS 6

class LineSensor
{
protected:

    bool onLine = false;
    int32_t lastPosition = 0;
    bool prevOnIntersection = false;
    unsigned char numSensors = NUM_SENSORS;
    unsigned char numSamplesPerSensor = 4;
    unsigned char emitterPin = LINE_CONTROL_OUT;
    //QTRSensors qtr;
    const byte lineSensorPins[NUM_SENSORS] = {LINE_SENSOR_1, LINE_SENSOR_3, LINE_SENSOR_5, LINE_SENSOR_7, LINE_SENSOR_9, LINE_SENSOR_11};

    uint16_t sensorValues[NUM_SENSORS];

public:

    LineSensor(void) {}
    void Initialize(void);
    float CalcError(void);
    bool CheckIntersection(void);
    void Calibrate(void);
    void read(void);
    void readCalibration(void);
    float readLineBlack(void);
    void getCalibrationMinMax(void);
};