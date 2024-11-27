#include "robot.h"

#define WEIGHING_PIN A6

void Robot::enterWeighing(){
    lifterTimer.start(50);
    robotState = ROBOT_WEIGHING;
    lifterCounter = 0;
    chassis.Stop();
    Serial.println("-> Weighing");
}

bool Robot::checkWeighing(){
    if (lifterCounter >= NUM_WEIGHT_MEAS){
        return true;
    }
    return false;
}

void Robot::handleWeighing(){
    if (robotState == ROBOT_WEIGHING){
        /*
        #ifdef __LOAD_CELL_DEBUG__
        Serial.print("Final Weight: ");
        Serial.println(calculateWeight((weightSum / lifterCounter)));
        #endif
        */
        esp.sendMessage("bill/TagID" + String(camera.currTag.id), String(calculateWeight(weightSum / lifterCounter)));
        lifterCounter = 0;
        weightSum = 0;
        lifterTimer.cancel();
        EnterIdleState();
    }
}

float Robot::calculateWeight(float weightADC){
    return ((weightADC - 324) * 3);
}

void Robot::updateWeighing(){
    if (robotState == ROBOT_WEIGHING){
        if (lifterTimer.checkExpired(true)){
            if (lifterCounter < NUM_WEIGHT_MEAS){
                weightSum += analogRead(WEIGHING_PIN);
                lifterCounter += 1;
                #ifdef __LOAD_CELL_DEBUG__
                plotVariable("weight ADC", analogRead(WEIGHING_PIN));
                plotVariable("weight (g)", calculateWeight(analogRead(WEIGHING_PIN)));
                #endif
            }
        }
    }
}

void Robot::lowerArm(){
    lifterServo.setTargetPos(1700);
}
void Robot::raiseArm(){
    lifterServo.setTargetPos(570);
}