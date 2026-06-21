#ifndef DECISION_H
#define DECISION_H

#include "perception.h"
#include "actuation.h"

// Finite State Machine (FSM) states for navigation
typedef enum {
    STATE_GO_TO_GOAL,
    STATE_OBSTACLE_AVOIDANCE,
    STATE_GOAL_REACHED
} RobotState;

// Function to process the next logic state based on sensory data
void process_decision_making(SensorData sensors, RobotPosition position);

#endif // DECISION_H