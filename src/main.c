#include <stdio.h>
#include <unistd.h>
#include "webots_stub.h"
#include "actuation.h"

int main(void) {
    // 1. Inicializa a comunicação oficial com o simulador Webots (ou roda o Stub se não achar)
    wb_robot_init();
    
    printf("[TEST] Inicializando teste isolado de Actuation e Odometria...\n");
    init_motors_and_odometry();
    
    printf("[TEST] Movendo para a frente por 3 segundos...\n");
    set_motor_speeds(15, 15);
    
    // Simula o avanço do tempo e atualiza a odometria a cada 20ms
    for (int i = 0; i < 150; i++) {
        // wb_robot_step sincroniza o relógio do seu código com o do Webots
        if (wb_robot_step(20) == -1) break; 
        
        RobotPosition current_pos = update_odometry();
        printf("Posicao Atual -> X: %.3f metros | Y: %.3f metros | Theta: %.3f rad\n", 
               current_pos.x, current_pos.y, current_pos.theta);
    }
    
    printf("[TEST] Parando os motores...\n");
    stop_motors();
    
    wb_robot_cleanup(); // Encerra a conexão com o simulador
    return 0;
}