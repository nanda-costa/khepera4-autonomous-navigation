#include <stdio.h>
#include <unistd.h>
#include "perception.h"
#include "actuation.h"
#include "decision.h"

int main(void) {
    printf("[SYSTEM] Initializing Autonomous Navigation Software...\n");
    
    // Initialize low-level drivers and reset odometry track
    init_motors_and_odometry();
    
    // Synchronous real-time control loop
    while (1) {
        // 1. Perception Layer (Alberth Viana)
        SensorData sensors = read_and_filter_sensors();
        
        // 2. Localization Layer (Fernanda Costa)
        RobotPosition position = update_odometry();
        
        // 3. Decision & Actuation Layer (Matheus Reges & Alexandre)
        process_decision_making(sensors, position);
        
        // Control loop frequency: 20ms (50Hz) to prevent CPU overhead on embedded Linux
        usleep(20000); 
    }
    
    return 0;
}