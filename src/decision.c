#include <math.h>
#include <stdio.h>
#include "decision.h"

typedef enum {
    LOGICA_CONDUZIR,
    LOGICA_PIVO_DESVIO,
    LOGICA_ESCAPE,
    LOGICA_REALINHAR
} EstadoControle;

static EstadoControle estado_interno = LOGICA_CONDUZIR;
static double angulo_original = 0.0;
static int gravou_angulo = 0;
static int contador_escape = 0;

TargetVelocities process_control_logic(RobotPosition pose, SensorData sensores, double alvo_x, double alvo_y) {
    TargetVelocities velocities;
    
    // 1. MÁQUINA DE ESTADOS COMPORTAMENTAL (SEQUENCIAL SEQUER ATROPELADA)
    if (sensores.obstaculo_detectado == 1) {
        if (estado_interno != LOGICA_PIVO_DESVIO && gravou_angulo == 0) {
            angulo_original = pose.theta; 
            gravou_angulo = 1;
        }
        estado_interno = LOGICA_PIVO_DESVIO;
    } 
    else {
        // Se a pista limpou e ele estava no meio do desvio, ativa o escape obrigatório
        if (estado_interno == LOGICA_PIVO_DESVIO) {
            estado_interno = LOGICA_ESCAPE;
            contador_escape = 0; 
        }
    }

    // 2. LÓGICA DE EXECUÇÃO E VELOCIDADES
    switch (estado_interno) {
        
        case LOGICA_CONDUZIR: {
            double v_linear = 10.0; 
            velocities.left_velocity  = v_linear;
            velocities.right_velocity = v_linear;
            velocities.estado_atual   = IR_PARA_ALVO; // Estado 0
            gravou_angulo = 0;
            break;
        }
        
        case LOGICA_PIVO_DESVIO: {
            if (sensores.forca_desvio >= 0) {
                velocities.left_velocity  =  20.0;
                velocities.right_velocity = -20.0; 
            } else {
                velocities.left_velocity  = -20.0;
                velocities.right_velocity =  20.0;
            }
            velocities.estado_atual = DESVIAR_OBSTACULO; // Estado 1
            break;
        }
        
        case LOGICA_ESCAPE: {
            // Avança em linha reta por 60 ciclos de forma blindada para ultrapassar o bloco
            if (contador_escape++ < 60) {
                velocities.left_velocity  = 10.0;
                velocities.right_velocity = 10.0;
                velocities.estado_atual   = IR_PARA_ALVO; // Mantém Estado 0 no print
            } else {
                // Passou a resma completamente! Agora sim libera para alinhar
                estado_interno = LOGICA_REALINHAR; 
            }
            break;
        }
        
        case LOGICA_REALINHAR: {
            double erro_angulo = angulo_original - pose.theta;
            
            while (erro_angulo >  M_PI) erro_angulo -= 2.0 * M_PI;
            while (erro_angulo < -M_PI) erro_angulo += 2.0 * M_PI;

            if (fabs(erro_angulo) < 0.05) {
                // Alinhamento cravado com precisão de bússola
                estado_interno = LOGICA_CONDUZIR; 
                velocities.left_velocity  = 10.0;
                velocities.right_velocity = 10.0;
                velocities.estado_atual   = IR_PARA_ALVO;
            } else {
                // Pivô suave de retorno ao rumo original
                double ganho_giro = 20.0 * erro_angulo;
                if (ganho_giro >  16.0) ganho_giro =  16.0;
                if (ganho_giro < -16.0) ganho_giro = -16.0;

                velocities.left_velocity  = -ganho_giro;
                velocities.right_velocity =  ganho_giro;
                velocities.estado_atual   = PARADO; // Estado 2
            }
            break;
        }
    }

    return velocities;
}