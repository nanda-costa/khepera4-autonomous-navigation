#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <stdbool.h>
#include <signal.h>

// --- DECLARES OF HARDWARE (Colocado no topo para resolver o erro de tipo desconhecido) ---
#ifndef ROBOT_FISICO
    #include "webots_stub.h"
    #include <webots/robot.h>
    #include <webots/motor.h>
    #include <webots/position_sensor.h>
    #include <webots/distance_sensor.h>
#else
    #include <khepera/khepera4.h>
#endif

// Caso a biblioteca oficial não defina, garantimos o tipo aqui
typedef struct knet_dev_s knet_dev_t;
extern knet_dev_t * dsPic;

// --- TRATAMENTO DO CTRL+C CORRIGIDO E SEGURO ---
void tratar_ctrl_c(int sinal) {
    printf("\n[CTRL+C] Interrupcao detectada! Forcando parada dos motores fisicos...\n");
    
    if (dsPic != NULL) {
        // 1. Envia velocidade zero para ambas as rodas imediatamente no registrador
        kh4_set_speed(0, 0, dsPic);
        
        // 2. Aguarda um pequeno instante para garantir que os pacotes I2C foram gravados no hardware
        usleep(50000); 
    }
    
    printf("[CTRL+C] Motores parados com sucesso. Encerrando processo Linux.\n");
    _exit(0); // Agora sim encerra o programa com segurança
}

// --- CABEÇALHOS DO SEU PROJETO ANTIGO ---
#include "actuation.h"
#include "perception.h"
#include "decision.h"

#define TIME_STEP 32
#define MAX_SPEED 47.6       
#define CALIBRATION_MODE 0

int main() {
    signal(SIGINT, tratar_ctrl_c);
    
    #ifndef ROBOT_FISICO
        wb_robot_init();
        printf("[MAIN] Inicializando hardware simulado no Webots...\n");
        init_motors_and_odometry();
    #else
        printf("[MAIN] Inicializando drivers físicos da libkh4 no ARM...\n");
        init_motors_and_odometry();
    #endif

    double alvo_x = 1.0;
    double alvo_y = 1.0;
    printf("Missão Iniciada: Alvo em (%.1f, %.1f)\n", alvo_x, alvo_y);

    while (1) {
        RobotPosition pose_atual = update_odometry(); 

        // FIX DE ALINHAMENTO DE MEMÓRIA: Usamos o tipo correto esperado pelo hardware
        int sensores_buffer[12] = {0};
        unsigned short sensores_reais[12] = {0};
        
        if (dsPic != NULL) {
            // A função oficial do robô preenche o buffer com inteiros limpos do dsPic
            kh4_proximity_ir(sensores_buffer, dsPic);
        }
        
        // Copia e mascara os valores garantindo apenas os bits válidos do conversor AD (0 a 1023)
        for (int i = 0; i < 12; i++) {
            sensores_reais[i] = (unsigned short)(sensores_buffer[i] & 0x03FF);
        }

        if (CALIBRATION_MODE) {
            static int contador_calibracao = 0;
            if (contador_calibracao++ % 10 == 0) {
                printf("[RAW]   0:%d 1:%d 2:%d 3:%d 4:%d 5:%d 6:%d 7:%d 8:%d 9:%d 10:%d 11:%d\n",
                       sensores_buffer[0],
                       sensores_buffer[1],
                       sensores_buffer[2],
                       sensores_buffer[3],
                       sensores_buffer[4],
                       sensores_buffer[5],
                       sensores_buffer[6],
                       sensores_buffer[7],
                       sensores_buffer[8],
                       sensores_buffer[9],
                       sensores_buffer[10],
                       sensores_buffer[11]);
                printf("[LIMPO] 0:%d 1:%d 2:%d 3:%d 4:%d 5:%d 6:%d 7:%d 8:%d 9:%d 10:%d 11:%d\n",
                       sensores_reais[0],
                       sensores_reais[1],
                       sensores_reais[2],
                       sensores_reais[3],
                       sensores_reais[4],
                       sensores_reais[5],
                       sensores_reais[6],
                       sensores_reais[7],
                       sensores_reais[8],
                       sensores_reais[9],
                       sensores_reais[10],
                       sensores_reais[11]);
            }

            set_motor_speeds(0, 0);
            usleep(TIME_STEP * 1000);
            continue;
        }

        // Passa o array perfeitamente alinhado e sem lixo para o filtro original processar
        SensorData dados_sensores = read_and_filter_sensors(sensores_reais);
        TargetVelocities comandos = process_control_logic(pose_atual, dados_sensores, alvo_x, alvo_y);

        if (comandos.left_velocity > MAX_SPEED)   comandos.left_velocity = MAX_SPEED;
        if (comandos.left_velocity < -MAX_SPEED)  comandos.left_velocity = -MAX_SPEED;
        if (comandos.right_velocity > MAX_SPEED)  comandos.right_velocity = MAX_SPEED;
        if (comandos.right_velocity < -MAX_SPEED) comandos.right_velocity = -MAX_SPEED;

        // Telemetria controlada com os valores limpos (0 a 1023)
        static int contador_print = 0;
        if (contador_print++ % 10 == 0) {
            printf("[STATUS] Estado: %d | IR0: %d | IR1: %d | IR2: %d | obst: %d | desvio: %.1f | L: %.1f | R: %.1f\n",
                   comandos.estado_atual,
                   sensores_reais[0],
                   sensores_reais[1],
                   sensores_reais[2],
                   dados_sensores.obstaculo_detectado,
                   dados_sensores.forca_desvio,
                   comandos.left_velocity,
                   comandos.right_velocity);
        }

        set_motor_speeds((int)comandos.left_velocity, (int)comandos.right_velocity);

        usleep(TIME_STEP * 1000); 
    }
}
