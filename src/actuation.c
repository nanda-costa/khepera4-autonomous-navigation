#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h> 
#include <khepera/khepera4.h> 
#include "actuation.h"
#include <signal.h>

#define PULSES_PER_MM  147.453 
#define WHEEL_BASE_MM  105.40
#define MM_TO_METERS   0.001

static RobotPosition current_position = {0.0, 0.0, 0.0};
static int last_left_encoder = 0;
static int last_right_encoder = 0;

knet_dev_t * dsPic;

void init_motors_and_odometry(void) {
    printf("[ACTUATION] Initializing physical low-level motor drivers...\n");
    if (kh4_init(0, NULL) < 0) {
        printf("[ERROR] Failed to initialize Khepera IV Hardware!\n");
        return;
    }
    dsPic = knet_open("Khepera4:dsPic", KNET_BUS_ANY, NULL, NULL);
    if (dsPic == NULL) dsPic = knet_open("kh4:dspic", KNET_BUS_ANY, NULL, NULL);
    if (dsPic == NULL) dsPic = knet_open("i2c:2", KNET_BUS_ANY, NULL, NULL);

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

    int delta_left = current_left_enc - last_left_encoder;
    int delta_right = current_right_enc - last_right_encoder;

    last_left_encoder = current_left_enc;
    last_right_encoder = current_right_enc;

    double dl = (double)delta_left / PULSES_PER_MM;
    double dr = (double)delta_right / PULSES_PER_MM;
    double dc = 0.5 * (dl + dr);
    double dth = (dr - dl) / WHEEL_BASE_MM;

    // Acumula X, Y e Theta reais (essencial para a matemática do Webots funcionar!)
    current_position.x += dc * cos(current_position.theta + 0.5 * dth);
    current_position.y += dc * sin(current_position.theta + 0.5 * dth);
    current_position.theta += dth;

    while (current_position.theta >  M_PI) current_position.theta -= 2.0 * M_PI;
    while (current_position.theta < -M_PI) current_position.theta += 2.0 * M_PI;

    return current_position;
}

void set_motor_speeds(int left_speed, int right_speed) {
    if (dsPic != NULL) {
        int speed_L = left_speed; 
        int speed_R = right_speed;
        if (speed_L > 1000)  speed_L = 1000;
        if (speed_L < -1000) speed_L = -1000;
        if (speed_R > 1000)  speed_R = 1000;
        if (speed_R < -1000) speed_R = -1000;
        kh4_set_speed(speed_L, speed_R, dsPic);
    }
}