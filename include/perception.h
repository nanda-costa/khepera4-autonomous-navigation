#ifndef PERCEPTION_H
#define PERCEPTION_H

typedef struct {
    double front_distance;
    double left_distance;
    double right_distance;
} SensorData;

SensorData read_and_filter_sensors(void);

#endif