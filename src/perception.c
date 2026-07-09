#include <stdio.h>
#include <khepera/khepera4.h> // Include físico
#include "perception.h"

extern knet_dev_t * dsPic; // Puxa o descritor aberto lá no actuation.c

void init_sensors(void) {
    printf("[PERCEPTION] Initializing physical infrared/ultrasound sensors...\n");
}

void read_sensors(int *sensor_values) {
    kh4_proximity_ir(sensor_values, dsPic); 
}

// ADICIONE ESTA FUNÇÃO QUE ESTAVA FALTANDO PARA A MAIN PODER LINKAR:
SensorData read_and_filter_sensors(unsigned short us_sensors[12]) {
    SensorData dados;

    dados.front_distance = (double)us_sensors[1]; 
    dados.left_distance  = (double)us_sensors[0];
    dados.right_distance = (double)us_sensors[2];
    
    // Calibracao real:
    // IR1 ficou em ~15 sem resma e subiu para ~70 com a resma perto parada.
    // Em movimento, IR2 foi o sensor que subiu forte, de ~400 ate 1020.
    if (us_sensors[1] > 350 || us_sensors[0] > 290 || us_sensors[2] > 290) {
        dados.obstaculo_detectado = 1;
        dados.forca_desvio = dados.left_distance - dados.right_distance;
    } else {
        dados.obstaculo_detectado = 0;
        dados.forca_desvio = 0.0;
    }

    return dados;
}
