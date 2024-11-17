#pragma once
#include "chassis.h"
#include <LineSensor.h>
#include <LSM6.h>
#include <esp32.h>
#include <openmv.h>

class Robot
{
protected:
    /**
     * We define some modes for you. SETUP is used for adjusting gains and so forth. Most
     * of the activities will run in AUTO. You shouldn't need to mess with these.
     */
    enum ROBOT_CTRL_MODE
    {
        CTRL_TELEOP,
        CTRL_AUTO,
        CTRL_SETUP,
        CTRL_CALIBRATING
    };
    ROBOT_CTRL_MODE robotCtrlMode = CTRL_AUTO;

    /**
     * robotState is used to track the current task of the robot. You will add new states as 
     * the term progresses.
     */
    enum ROBOT_STATE 
    {
        ROBOT_IDLE, 
        ROBOT_LINING,
        ROBOT_TURNING,
        ROBOT_MOVE_DISTANCE,
        ROBOT_SEARCHING,
        ROBOT_APPROACHING
    };

    ROBOT_STATE robotState = ROBOT_IDLE;
    bool onUpRamp = false;
    bool onDownRamp = false;
    bool dropTrash = false;

    /* Define the chassis*/
    Chassis chassis;

    /* Line sensor */
    LineSensor lineSensor;
    float lineSum = 0;
    float prevLineError = 0;
    float Kp_line_fast = 30;
    float Ki_line_fast = 0.5;
    float Kd_line_fast = 10;
    float Kp_line_medium = 10;
    float Kd_line_medium = 6;
    float Kp_line_slow = 2;
    float Kd_line_slow = 1;

    /*Turning PID*/
    float Kp_turn = 4; //original: 5.2
    float Kd_turn  = 4;
    float Ki_turn = 0.01;
    float turnErrorSum = 0;
    float turnPrevError = 0;
    const float TURN_THRESHOLD = 1;

    /* Approach PID*/
    float Kp_approach = 0.1;
    float Kd_approach = 0;
    float Ki_approach = 0;
    float PrevApproachError = 0;
    float approachErrorSum = 0;


    /* To add later: rangefinder, camera, etc.*/

    /* ESP32 */
    ESP32 esp;

    /* Camera */
    OpenMV camera;

    // For managing key presses
    String keyString;

    /**
     * The LSM6 IMU that is included on the Romi. We keep track of the orientation through
     * Euler angles (roll, pitch, yaw).
     */
    LSM6 imu;
    LSM6::vector<float> prevEulerAngles;
    LSM6::vector<float> eulerAngles;
    LSM6::vector<float> observedEulerAngles;
    const float KAPPA = 0.015;

    /* targetHeading is used for commanding the robot to turn */
    float targetHeading = 0;

    /* baseSpeed is used to drive at a given speed while, say, line following.*/
    float baseSpeed = 25;
    float tempSpeedHolder = 0;

    int8_t iGrid = 0, jGrid = 0;
    int8_t direction = 0; //NORTH -> 0; WEST -> 1; SOUTH -> 2; EAST -> 3;
    // use direction %= 4 to account for loop arounds (direction overflow);

    int8_t iTargetInital = 2;
    int8_t jTargetInital = 2;
    int8_t iTarget = iTargetInital;
    int8_t jTarget = jTargetInital;

    float moveDistance;

    float rampUpAngleThreshold = -10;
    float rampDownAngleThreshold = 10;

    float tagZTransThreshold = 50;
    uint8_t tagUpdateTimer = 0;



    /**
     * For tracking the motion of the Romi. We keep track of the intersection we came
     * from and the one we're headed to. You'll program in the map in handleIntersection()
     * and other functions.
     */
    enum INTERSECTION {NODE_START, NODE_1, NODE_2, NODE_3,};
    INTERSECTION nodeFrom = NODE_START;
    INTERSECTION nodeTo = NODE_1;

    
public:
    Robot(void) {keyString.reserve(8);} //reserve some memory to avoid reallocation
    void InitializeRobot(void);
    void RobotLoop(void);

protected:
    /* For managing IR remote key presses*/
    void HandleKeyCode(int16_t keyCode);

    /* State changes */    
    void EnterIdleState(void);

    /* Mode changes */
    void EnterTeleopMode(void);
    void EnterAutoMode(void);
    void EnterSetupMode(void);

    /**
     * Line following and navigation routines.
     */
    void EnterLineFollowing();
    void EnterLineFollowing(int speed);
    void LineFollowingUpdate(void);

    void EnterCalibrating(void);
    void UpdateCalibration(void);

    bool CheckIntersection(void) {return lineSensor.CheckIntersection();}
    void HandleIntersection(void);

    void EnterTurn(int numTurns);
    bool CheckTurnComplete(void);
    void HandleTurnComplete(void);
    void TurningUpdate(void);
    void CalculateIntersection(void);
    
    bool checkUpRamp(void);
    void handleOnUpRamp(void);
    void handleOffUpRamp(void);
    bool checkDownRamp(void);
    void handleOnDownRamp(void);
    void handleOffDownRamp(void);

    bool checkMoving(void);
    void handleMovingComplete(void);
    void enterMoving(float distance);
    void updateMoving(void);

    void enterSearching(void);
    bool checkSearch(void);
    void handleSearchComplete(void);

    void enterApproaching(void);
    bool checkApproached(void);
    void handleApproachedComplete(void);
    void updateApproach(void);

    /* IMU routines */
    void HandleOrientationUpdate(void);

    /* For commanding the lifter servo */
    void SetLifter(uint16_t position);


    void plotVariable(String name, double var);
};