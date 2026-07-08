#include <math.h>
#include <stdio.h>
#include "decision.h"

static EstadoRobo estado_atual = IR_PARA_ALVO;

TargetVelocities process_control_logic(RobotPosition pose, SensorData sensores, double alvo_x, double alvo_y) {
    TargetVelocities velocities;
    
    // Atualiza a máquina de estados reativa
    if (sensores.obstaculo_detectado == 1) {
        estado_atual = DESVIAR_OBSTACULO;
    } else {
        estado_atual = IR_PARA_ALVO;
    }

    velocities.estado_atual = estado_atual;

    switch (estado_atual) {
        case IR_PARA_ALVO: {
            // Estado 0: Marcha para frente limpa e constante
            double v_linear = 6.0; 
            velocities.left_velocity  = v_linear;
            velocities.right_velocity = v_linear;
            break;
        }
        
        case DESVIAR_OBSTACULO: {
            // Estado 1: EVITAÇÃO AGRESSIVA NO PRÓPRIO EIXO (Giro de pivô)
            // Forçamos sinais estritamente opostos com velocidade forte (20.0) 
            // para o robô rotacionar instantaneamente no lugar antes de colidir
            double vel_rotacao = 20.0; 
            
            // Baseado no gradiente de desvio calculado pelo filtro da percepção
            if (sensores.forca_desvio >= 0) {
                // Obstáculo mais forte na esquerda -> Gira para a direita no eixo
                velocities.left_velocity  = vel_rotacao;
                velocities.right_velocity = -vel_rotacao; 
            } else {
                // Obstáculo mais forte na direita -> Gira para a esquerda no eixo
                velocities.left_velocity  = -vel_rotacao;
                velocities.right_velocity = vel_rotacao;
            }
            break;
        }
        
        case PARADO:
            velocities.left_velocity  = 0.0;
            velocities.right_velocity = 0.0;
            break;
    }

    return velocities;
}