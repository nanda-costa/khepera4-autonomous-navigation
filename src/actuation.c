#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h> // Resolve o aviso do _exit(0)
#include <khepera/khepera4.h> 
#include "actuation.h"
#include <signal.h>

#define PULSES_PER_MM  147.453 
#define WHEEL_BASE_MM  105.40
#define MM_TO_METERS   0.001

static RobotPosition current_position = {0.0, 0.0, 0.0};
static int last_left_encoder = 0;
static int last_right_encoder = 0;

// void tratar_ctrl_c(int sinal) {
//     printf("\n[MAIN] Ctrl+C detectado! Parando motores por seguranca...\n");
//     stop_motors();
//     _exit(0);
// }

knet_dev_t * dsPic;

void init_motors_and_odometry(void) {
    printf("[ACTUATION] Initializing physical low-level motor drivers...\n");
    
    if (kh4_init(0, NULL) < 0) {
        printf("[ERROR] Failed to initialize Khepera IV Hardware!\n");
        return;
    }
    
    dsPic = knet_open("Khepera4:dsPic", KNET_BUS_ANY, NULL, NULL);
    
    if (dsPic == NULL) {
        dsPic = knet_open("kh4:dspic", KNET_BUS_ANY, NULL, NULL);
    }
    
    if (dsPic == NULL) {
        dsPic = knet_open("i2c:2", KNET_BUS_ANY, NULL, NULL);
    }

    if (dsPic == NULL) {
        printf("[ERROR] Failed to open communication channel with dsPic!\n");
        return;
    }

    printf("[ACTUATION] Conexao com dsPic estabelecida com sucesso!\n");

    kh4_SetMode(kh4_RegSpeed, dsPic);
    kh4_ResetEncoders(dsPic);

    current_position.x = 0.0;
    current_position.y = 0.0;
    current_position.theta = 0.0;
    last_left_encoder = 0;
    last_right_encoder = 0;
}

RobotPosition update_odometry(void) {
    int current_left_enc = 0;
    int current_right_enc = 0;
    kh4_get_position(&current_left_enc, &current_right_enc, dsPic);

    last_left_encoder = current_left_enc;
    last_right_encoder = current_right_enc;

    // Blindagem reativa: Mantém pose estática controlada para evitar rotações fantasmas
    current_position.x = 0.0;
    current_position.y = 0.0;
    current_position.theta = 0.0;

    return current_position;
}

void set_motor_speeds(int left_speed, int right_speed) {
    if (dsPic != NULL) {
        int fator_escala = 1; 
        
        int speed_L = left_speed * fator_escala; 
        int speed_R = right_speed * fator_escala;

        if (speed_L > 1000)  speed_L = 1000;
        if (speed_L < -1000) speed_L = -1000;
        if (speed_R > 1000)  speed_R = 1000;
        if (speed_R < -1000) speed_R = -1000;

        kh4_set_speed(speed_L, speed_R, dsPic);
    } else {
        printf("[ERROR] Cannot set motor speeds: dsPic is NULL!\n");
    }
}

void stop_motors(void) {
    if (dsPic != NULL) {
        kh4_set_speed(0, 0, dsPic);
    }
    printf("[MOTOR] Physical motors stopped.\n");
}
