#ifndef DECISION_H
#define DECISION_H

#include "perception.h"
#include "actuation.h"

typedef enum {
    STATE_GO_TO_GOAL,
    STATE_OBSTACLE_AVOIDANCE,
    STATE_GOAL_REACHED
} RobotState;

void process_decision_making(SensorData sensors, RobotPosition position);

#endif