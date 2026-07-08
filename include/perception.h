#ifndef PERCEPTION_H
#define PERCEPTION_H

#define NUM_US_SENSORS 5

typedef struct {
    double front_distance;
    double left_distance;
    double right_distance;
    double forca_desvio;
    int obstaculo_detectado;
} SensorData;

SensorData read_and_filter_sensors(unsigned short us_sensors[12]);

#endif