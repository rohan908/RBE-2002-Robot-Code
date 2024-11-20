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
    turnErrorSum = 0;
    turnPrevError = 0;
}

void Robot::TurningUpdate(void){
    float error = (targetHeading - eulerAngles.z) * PI / 180;
    float turnEffort = Kp_turn * error + Kd_turn * (error - turnPrevError) + Ki_turn * turnErrorSum;
    chassis.SetTwist(0, turnEffort);
    turnPrevError = error;
    turnErrorSum += error;
}

bool Robot::CheckTurnComplete(void)
{
    bool retVal = false;

    if (fabs(targetHeading - eulerAngles.z) < TURN_THRESHOLD){
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
    if (robotState == ROBOT_TURNING && robotCtrlMode == CTRL_AUTO){
        EnterLineFollowing();
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
    LSM6::vector<float> observedAngles;
    LSM6::vector<float> predictedAngles;
    LSM6::vector<float> innnovationInDegrees;
    //observed
    observedAngles.y = degrees(atan2(-1 * (imu.a.x), (imu.a.z)));
    observedAngles.x = degrees(atan2(-1 * imu.a.y, imu.a.z));

    //predicted
    predictedAngles.x = prevEulerAngles.x + (imu.mdpsPerLSB / 1000) * (imu.g.x - imu.compBias.x) / imu.gyroODR; //divide the ODR since its in Hz
    predictedAngles.y = prevEulerAngles.y + (imu.mdpsPerLSB / 1000) * (imu.g.y - imu.compBias.y) / imu.gyroODR; //divide by 1000 to convert millidegrees ps to degrees ps

    innnovationInDegrees.y = observedAngles.y - predictedAngles.y;
    innnovationInDegrees.x = observedAngles.x - predictedAngles.x;
    if(robotState == ROBOT_IDLE)
    {
        // TODO: You'll need to add code to LSM6 to update the bias
        imu.updateGyroBias();
        //imu.updateAccelBias();
    }
    else // update orientation
    {
        imu.updateComplementaryBias(innnovationInDegrees);
        //corrected
        eulerAngles.x = predictedAngles.x + KAPPA * innnovationInDegrees.x;
        eulerAngles.y = predictedAngles.y + KAPPA * innnovationInDegrees.y;
        eulerAngles.z = prevEulerAngles.z + (imu.mdpsPerLSB / 1000)* (imu.g.z - imu.gyroBias.z) / imu.gyroODR;
    }

#ifdef __IMU_DEBUG__
    /*
    Serial.print(">Accel Yaw:");
    Serial.println(imu.a.z * imu.mgPerLSB / 1000);
    Serial.print(">Accel Roll:");
    Serial.println(imu.a.x* imu.mgPerLSB / 1000);
    Serial.print(">Accel Pitch:");
    Serial.println(imu.a.y* imu.mgPerLSB / 1000);
    */

    plotVariable("Observed Pitch", observedAngles.y);
    plotVariable("Predicted Pitch", predictedAngles.y);

    //plotVariable("X Accel Bias", imu.accelBias.x);
    //plotVariable("Y Accel Bias", imu.accelBias.y);
    plotVariable("Y Comp Bias", imu.compBias.y);

    /*
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
void Robot::EnterLineFollowing() 
{
    Serial.println(" -> LINING"); 
    robotState = ROBOT_LINING;
    lineSum = 0;
}

void Robot::EnterLineFollowing(int speed){
    Serial.println(" -> LINING"); 
    robotState = ROBOT_LINING;
    baseSpeed = speed > 0 ? speed : baseSpeed;
    lineSum = 0;
}

void Robot::LineFollowingUpdate(void)
{
    // TODO: calculate the error in CalcError(), calc the effort, and update the motion
    float lineError = lineSensor.CalcError();

    float Kp = Kp_line_fast;
    float Kd = Kd_line_fast;
    if (baseSpeed < 41){
        Kp = Kp_line_medium;
        Kd = Kd_line_medium;
    }
    if (baseSpeed < 35){
        Kp = Kp_line_slow;
        Kd = Kd_line_slow;
    }
    float turnEffort = Kp * lineError + Kd * (lineError - prevLineError); /*+ Ki_line * lineSum*/
    prevLineError = lineError;
    //lineSum += lineError;
    #ifdef __LINE_FOLLOW_DEBUG__
    plotVariable("baseSpeed", baseSpeed);
    plotVariable("Line Error", lineError);
    #endif
    chassis.SetTwist(baseSpeed, turnEffort);
}


void Robot::UpdateCalibration(void){
    if (robotCtrlMode == CTRL_CALIBRATING){
        lineSensor.Calibrate();
    }
}

/**
 * As coded, HandleIntersection will make the robot drive out 3 intersections, turn around,
 * and stop back at the start. You will need to change the behaviour accordingly.
 */
void Robot::HandleIntersection(void)
{
    if (robotState == ROBOT_LINING && !onUpRamp && !onDownRamp){
        #ifdef __INTERSECTION_HANDLING_DEBUG__
        Serial.println("Found Intersection.");
        #endif
        enterMoving(8.5);
    }
}

void Robot::CalculateIntersection(){

    switch (direction){
        case 0:
            jGrid++;
            break;
        case 1:
            iGrid--;
            break;
        case 2:
            jGrid--;
            break;
        case 3:
            iGrid++;
            break;
    }
    uint8_t targetDirection;
    if(iGrid == 0 && jGrid == 0 && iTarget == 0 && jTarget == 0){
        Serial.println("Returned Home");
        EnterIdleState();
    }
    else {
        if (jGrid == jTarget && iGrid == iTarget){
            iTarget = 0;
            jTarget = 0;
            switch (direction){
                case 0:
                    targetDirection = 2;
                    break;
                case 1:
                    targetDirection = 3;
                    break;
                case 2:
                    targetDirection = 0;
                    break;
                case 3:
                    targetDirection = 1;
                    break;
            }
        }
        else if (jGrid != jTarget){
            if (jTarget - jGrid < 0){
                targetDirection = 2; //go South
            }
            else {
                targetDirection = 0; //go North
            }
        
        }
        else if (iGrid != iTarget) {
            //if iTarget - iGrid == -1, then 
            if (iTarget - iGrid < 0){
                targetDirection = 1; //go West
            }
            else {
                targetDirection = 3; //go East
            }
        }

        if (targetDirection != direction){
            int numTurns = targetDirection - direction;
            numTurns = (fabs(numTurns) > 2) ? -1 * numTurns % 2 : numTurns;
            
            #ifdef __INTERSECTION_HANDLING_DEBUG__
            Serial.print("numTurns requested at intersection: ");
            Serial.println(numTurns);
            plotVariable("iGrid", iGrid);
            plotVariable("jGrid", jGrid);
            plotVariable("target iGrid", iTarget);
            plotVariable("target jGrid", jTarget);
            plotVariable("direction", direction);
            plotVariable("targetDirection", targetDirection);
            #endif
            EnterTurn(numTurns);
        }
        else{
            EnterLineFollowing();
        }
    }
}


bool Robot::checkMoving(){
    bool retVal = false;
   // float error = chassis.calcDistanceError();
    if (chassis.checkDistance()){
        retVal = true;
    }
    #ifdef __INTERSECTION_HANDLING_DEBUG__
        plotVariable("iGrid", iGrid);
        plotVariable("jGrid", jGrid);
        plotVariable("target iGrid", iTarget);
        plotVariable("target jGrid", jTarget);
        plotVariable("direction", direction);
    #endif
    return retVal;
}

void Robot::handleMovingComplete(){
    if (robotState == ROBOT_MOVE_DISTANCE) {
        if (dropTrash){
            EnterTurn(2);
            dropTrash = false;
        }
        else{
            EnterIdleState();
            //CalculateIntersection();
        }
    }
}

void Robot::enterMoving(float distanceInCm){
    Serial.println("-> MOVING");
    robotState = ROBOT_MOVE_DISTANCE;
    moveDistance = distanceInCm;
    if (moveDistance < 0){
        baseSpeed = -1 * baseSpeed;
    }
    chassis.saveStartingEncoder();
    chassis.setTargetEncoderForDistance(distanceInCm);
    chassis.SetTwist(baseSpeed,0);
    targetHeading = eulerAngles.z;
    turnErrorSum = 0;
    turnPrevError = 0;
    
}

void Robot::updateMoving(){
    float error = (targetHeading - eulerAngles.z) * PI / 180;
    float turnEffort = Kp_turn * error + Kd_turn * (error - turnPrevError) + Ki_turn * turnErrorSum;
    chassis.SetTwist(baseSpeed, turnEffort);
    turnPrevError = error;
    turnErrorSum += error;
}

bool Robot::checkUpRamp(){
    bool retVal = false;
    if(eulerAngles.y < rampUpAngleThreshold){
        retVal = true;
    }
    return retVal;
}

void Robot::handleOnUpRamp(){
    if (!onUpRamp){
        onUpRamp = true;
        Serial.println("-> On Up Ramp");
        digitalWrite(13, HIGH);
        rampUpAngleThreshold = -3;
        tempSpeedHolder  = baseSpeed;
        baseSpeed = 10;
    }
}

void Robot::handleOffUpRamp(){
    if (onUpRamp){
        onUpRamp = false;
        Serial.println("-> Off Up Ramp");
        rampUpAngleThreshold = -10;
        digitalWrite(13, LOW);
        baseSpeed = tempSpeedHolder;
        dropTrash = true;
        enterMoving(10);
    }
}

bool Robot::checkDownRamp(){
    bool retVal = false;
    if(eulerAngles.y > rampDownAngleThreshold){
        retVal = true;
    }
    return retVal;
}

void Robot::handleOnDownRamp(){
    if (!onDownRamp){
        onDownRamp = true;
        dropTrash = false;
        Serial.println("-> On Down Ramp");
        digitalWrite(13, HIGH);
        rampDownAngleThreshold = 3;
    }
}

void Robot::handleOffDownRamp(){
    if (onDownRamp){
        onDownRamp = false;
        Serial.println("-> Off Down Ramp");
        rampDownAngleThreshold = 10;
        EnterIdleState();
        digitalWrite(13, LOW);
    }
}

void Robot::enterSearching(){
    chassis.SetTwist(0, 0.5);
    Serial.println("-> SEARCHING");
    robotState = ROBOT_SEARCHING;
}

bool Robot::checkSearch(){
    bool retVal = false;
    if (camera.getTagCount()){
        retVal = true;
    }
    return retVal;
}

void Robot::handleSearchComplete(){
    if (robotState == ROBOT_SEARCHING){
        chassis.SetTwist(0,0);
        enterApproaching();
    }
}

void Robot::enterApproaching(){
    camera.readTag(camera.tag);
    char str[3];
    sprintf(str, "%d", camera.currTag.id);
    esp.sendMessage("CurrentTag", str);
    robotState = ROBOT_APPROACHING;
    Serial.println("-> APPROACHING");
}

bool Robot::checkApproached(){
    if (camera.seesTag && (camera.currTag.z < 3)){
        startTime = 1;
        return true;
    }
    if (startTime){
        startTime = millis();
    }
    return false;
}

void Robot::updateApproach(){
    if (tagUpdateTimer % 3 == 0){
        char str[3];
        sprintf(str, "%d", camera.currTag.id);
        esp.sendMessage("CurrentTag", str);
        camera.handleTags();
    }
    tagUpdateTimer++;
    float error = camera.currTag.cx;
    float turnError = Kp_approach * error + Kd_approach * (error - PrevApproachError) + Ki_approach * approachErrorSum;
    PrevApproachError = error;
    approachErrorSum += error;
    #ifdef __APPROACH_DEBUG__
        plotVariable("tagError", error);
    #endif
    chassis.SetTwist(-1 * 5, -1 * turnError); //multiply -1 since the romi is backwards
}

void Robot::handleApproachedComplete(){
    if(robotState == ROBOT_APPROACHING){
        enterMoving(-7);
    }
}

void Robot::handleLostTag(){
    if(robotState == ROBOT_APPROACHING && !camera.seesTag){
        if ((millis() - startTime) > 1000){
            enterSearching();
        }
    }
}



void Robot::plotVariable(String name, double variable){
    Serial.print(">");
    Serial.print(name);
    Serial.print(":"); 
    Serial.println(variable);
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
    camera.PrintAprilTags();

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
            //Serial.println("i'm in lining update loop");
        }

        if(robotState == ROBOT_TURNING){
            TurningUpdate();
        }

        if (robotCtrlMode == CTRL_CALIBRATING){
            UpdateCalibration();
        }

        if(robotState == ROBOT_MOVE_DISTANCE){
            updateMoving();
        }

        if(robotState == ROBOT_APPROACHING){
            updateApproach();
        }


        chassis.UpdateMotors();

        // add synchronous, post-motor-update actions here

    }
    //if (robotCtrlMode == CTRL_CALIBRATING) UpdateCalibration();

    /**
     * Check for any intersections
     */
    if(lineSensor.CheckIntersection()) HandleIntersection();

    if(checkMoving()) handleMovingComplete();

    if(checkUpRamp()){
        handleOnUpRamp();
    }
    else{
        handleOffUpRamp();
    }
    if(checkDownRamp()){
        handleOnDownRamp();
    }
    else{
        handleOffDownRamp();
    }
    if(checkApproached()) {
        handleApproachedComplete();
    }
    else{
        handleLostTag();
    }

    if(checkSearch()) handleSearchComplete();

    /**
     * Check for an IMU update
     */
    //digitalWrite(4, HIGH);
    if(imu.checkForNewData())
    {
        HandleOrientationUpdate();
        if(CheckTurnComplete()) HandleTurnComplete();
    }
    //digitalWrite(4, LOW);
}

