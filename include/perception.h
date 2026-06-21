#ifndef PERCEPTION_H
#define PERCEPTION_H

// Structure to store clean and filtered sensor readings
typedef struct {
    double front_distance;
    double left_distance;
    double right_distance;
} SensorData;

// Function to read and filter the physical sensors
SensorData read_and_filter_sensors(void);

#endif // PERCEPTION_H