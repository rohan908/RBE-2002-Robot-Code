#include "robot.h"
#include <IRdecoder.h>
#include <FastGPIO.h>

void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();

    /**
     * Initialize the IR decoder. Declared extern in IRdecoder.h; see robot-remote.cpp
     * for instantiation and setting the pin.
     */
    decoder.init();

    /**
     * Initialize the IMU and set the rate and scale to reasonable values.
     */
    imu.init();

    /**
     * TODO: Add code to set the data rate and scale of IMU (or edit LSM6::setDefaults())
     */

    // The line sensor elements default to INPUTs, but we'll initialize anyways, for completeness
    lineSensor.Initialize();
}

void Robot::EnterIdleState(void)
{
    Serial.println("-> IDLE");
    chassis.Stop();
    keyString = "";
    robotState = ROBOT_IDLE;
}

/**
 * Functions related to the IMU (turning; ramp detection)
 */
void Robot::EnterTurn(int numTurns)
{
    Serial.println(" -> TURN");
    robotState = ROBOT_TURNING;
    direction = (direction + numTurns) % 4;
    direction = direction < 0 ? direction + 4 : direction;
    targetHeading = (numTurns * 90) + eulerAngles.z;
}

void Robot::TurningUpdate(void){
    float error = (targetHeading - eulerAngles.z) * PI / 180;
    turnErrorSum += error;
    float turnEffort = Kp_turn * error + Kd_turn * (error - turnPrevError) + Ki_turn * turnErrorSum;
    chassis.SetTwist(0, turnEffort);
    turnPrevError = error;
}

bool Robot::CheckTurnComplete(void)
{
    bool retVal = false;

    if (fabs(targetHeading - eulerAngles.z) < 0.01){
        retVal = true;
    }

    #ifdef __MOTOR_DEBUG__
    Serial.print(">targetHeading:");
    Serial.println(targetHeading);
    #endif

    return retVal;
}

void Robot::HandleTurnComplete(void)
{
    if (robotState == ROBOT_TURNING){
        Serial.println(" -> IDLE");
        robotState = ROBOT_IDLE;
    }
}

/**
 * Here is a good example of handling information differently, depending on the state.
 * If the Romi is not moving, we can update the bias (but be careful when you first start up!).
 * When it's moving, then we update the heading.
 */
void Robot::HandleOrientationUpdate(void)
{
    prevEulerAngles = eulerAngles;
    if(robotState == ROBOT_IDLE)
    {
        // TODO: You'll need to add code to LSM6 to update the bias
        imu.updateGyroBias();
    }

    else // update orientation
    {
        eulerAngles.x = prevEulerAngles.x + (imu.mdpsPerLSB / 1000) * (imu.g.x - imu.gyroBias.x) / imu.gyroODR; //divide the ODR since its in Hz
        eulerAngles.y = prevEulerAngles.y + (imu.mdpsPerLSB / 1000) * (imu.g.y - imu.gyroBias.y) / imu.gyroODR; //divide by 1000 to convert millidegrees ps to degrees ps
        eulerAngles.z = prevEulerAngles.z + (imu.mdpsPerLSB / 1000)* (imu.g.z - imu.gyroBias.z) / imu.gyroODR;
    }

#ifdef __IMU_DEBUG__
    /*
    Serial.print(">Biased Yaw:");
    Serial.println(imu.g.z);
    Serial.print(">Biased Roll:");
    Serial.println(imu.g.x);
    Serial.print(">Biased Pitch:");
    Serial.println(imu.g.y);

    Serial.print(">UnBiased Yaw:");
    Serial.println(imu.g.z - imu.gyroBias.z);
    Serial.print(">UnBiased Roll:");
    Serial.println(imu.g.x - imu.gyroBias.x);
    Serial.print(">UnBiased Pitch:");
    Serial.println(imu.g.y - imu.gyroBias.y);

    Serial.print(">Yaw Bias:");
    Serial.println(imu.gyroBias.z);
    */



    Serial.print(">Yaw:");
    Serial.println(eulerAngles.z);
    Serial.print(">Roll:");
    Serial.println(eulerAngles.x);
    Serial.print(">Pitch:");
    Serial.println(eulerAngles.y);
#endif
}

/**
 * Functions related to line following and intersection detection.
 */
void Robot::EnterLineFollowing(float speed) 
{
    Serial.println(" -> LINING"); 
    baseSpeed = speed; 
    robotState = ROBOT_LINING;
    lineSum = 0;
}

void Robot::LineFollowingUpdate(void)
{
    if(robotState == ROBOT_LINING) 
    {
        // TODO: calculate the error in CalcError(), calc the effort, and update the motion
        float lineError = lineSensor.CalcError();
        float Kp = Kp_line;
        float Kd = Kd_line;
        if (baseSpeed < 41){
            Kp = Kp_line_slow;
            Kd = Kd_line_slow;
        }
        float turnEffort = Kp * lineError + Kd * (lineError - prevLineError); /*+ Ki_line * lineSum*/
        prevLineError = lineError;
        //lineSum += lineError;
        chassis.SetTwist(baseSpeed, turnEffort);
    }
}

/*
void Robot::UpdateCalibration(void){
    if (robotCtrlMode == CTRL_CALIBRATING){
        lineSensor.Calibrate();
    }
}*/

/**
 * As coded, HandleIntersection will make the robot drive out 3 intersections, turn around,
 * and stop back at the start. You will need to change the behaviour accordingly.
 */
void Robot::HandleIntersection(void)
{
    Serial.print("X:");
    
}

void Robot::RobotLoop(void) 
{
    /**
     * The main loop for your robot. Process both synchronous events (motor control),
     * and asynchronous events (IR presses, distance readings, etc.).
    */

    /**
     * Handle any IR remote keypresses.
     */
    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) HandleKeyCode(keyCode);

    /**
     * Check the Chassis timer, which is used for executing motor control
     */
    if(chassis.CheckChassisTimer())
    {
        // add synchronous, pre-motor-update actions here
        if(robotState == ROBOT_LINING)
        {
            LineFollowingUpdate();
        }

        if(robotState == ROBOT_TURNING){
            TurningUpdate();
        }


        chassis.UpdateMotors();

        // add synchronous, post-motor-update actions here

    }
    //if (robotCtrlMode == CTRL_CALIBRATING) UpdateCalibration();

    /**
     * Check for any intersections
     */
    if(lineSensor.CheckIntersection()) HandleIntersection();

    /**
     * Check for an IMU update
     */
    if(imu.checkForNewData())
    {
        HandleOrientationUpdate();
        if(CheckTurnComplete()) HandleTurnComplete();
    }
}

