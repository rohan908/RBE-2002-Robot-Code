#include "Romi32U4MotorTemplate.h"
#include <PCint.h>

void leftISR(void)
{
    leftMotor.ProcessEncoderTick();
}

void rightISR(void)
{
    rightMotor.ProcessEncoderTick();
}

void Romi32U4MotorBase::AttachInterrupts(void) 
{
    attachPCInt(digitalPinToPCInterrupt(LEFT_XOR), leftISR);
    attachInterrupt(digitalPinToInterrupt(RIGHT_XOR), rightISR, CHANGE);
}
