#include <stdio.h>
#include <math.h>
#include "actuation.h"

#define PULSES_PER_MM  147.453
#define WHEEL_BASE_MM  105.40
#define MM_TO_METERS   0.001

// Variáveis globais internas para o rastreio da odometria
static RobotPosition current_position = {0.0, 0.0, 0.0};
static int last_left_encoder = 0;
static int last_right_encoder = 0;

void init_motors_and_odometry(void) {
    printf("[ACTUATION] Initializing low-level motor drivers...\n");
    
    current_position.x = 0.0;
    current_position.y = 0.0;
    current_position.theta = 0.0;
    
    last_left_encoder = 0;
    last_right_encoder = 0;
    
    printf("[ACTUATION] Odometry target reset to (0,0,0).\n");
}

RobotPosition update_odometry(void) {
    // Mantendo os encoders simulados em zero para o teste de loop estável
    int current_left_enc = 0;
    int current_right_enc = 0;
    
    // 1. Calcula a variação dos impulsos (ticks) desde a última leitura
    int delta_left = current_left_enc - last_left_encoder;
    int delta_right = current_right_enc - last_right_encoder;
    
    // 2. Converte os impulsos das rodas para deslocamento em milímetros
    double dist_left_mm = (double)delta_left / PULSES_PER_MM;
    double dist_right_mm = (double)delta_right / PULSES_PER_MM;
    
    // 3. Calcula o deslocamento linear central (dS) e a variação angular (dTheta)
    double delta_s_mm = (dist_left_mm + dist_right_mm) / 2.0;
    double delta_theta = (dist_right_mm - dist_left_mm) / WHEEL_BASE_MM;
    
    // 4. Integra os novos valores na posição global (convertendo para metros)
    current_position.x += (delta_s_mm * cos(current_position.theta)) * MM_TO_METERS;
    current_position.y += (delta_s_mm * sin(current_position.theta)) * MM_TO_METERS;
    current_position.theta += delta_theta;
    
    // Restringe o ângulo Theta entre -PI e +PI (Normalização)
    if (current_position.theta > M_PI)  current_position.theta -= 2.0 * M_PI;
    if (current_position.theta < -M_PI) current_position.theta += 2.0 * M_PI;
    
    // Atualiza o histórico para o próximo ciclo
    last_left_encoder = current_left_enc;
    last_right_encoder = current_right_enc;
    
    return current_position;
}

void set_motor_speeds(int left_speed, int right_speed) {
    // LOG temporário para validação no terminal simulado
    printf("[MOTOR] Left: %d | Right: %d\n", left_speed, right_speed);
}

void stop_motors(void) {
    set_motor_speeds(0, 0);
    printf("[MOTOR] Stopped completely.\n");
}