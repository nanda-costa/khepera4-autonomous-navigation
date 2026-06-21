#ifndef ACTUATION_H
#define ACTUATION_H

typedef struct {
    double x;
    double y;
    double theta;
} RobotPosition;

void init_motors_and_odometry(void);
RobotPosition update_odometry(void);
void set_motor_speeds(int left_speed, int right_speed);
void stop_motors(void);

#endif