#ifndef DECISION_H
#define DECISION_H

#include "actuation.h"
#include "perception.h"

// Definição dos estados da FSM (Máquina de Estados)
typedef enum {
    IR_PARA_ALVO,
    DESVIAR_OBSTACULO,
    PARADO
} EstadoRobo;

// Estrutura para retornar os comandos de velocidade para a main
typedef struct {
    double left_velocity;
    double right_velocity;
    EstadoRobo estado_atual;

    // Campos de diagnóstico (não usados no controle, só telemetria)
    int fase_interna;        // 0=conduzindo, 1=girando p/ desviar, 2=escape reto pós-desvio
    double dist_ate_a_reta;  // distância perpendicular até a reta origem->alvo, em mm (só válido na fase 2)
    double dist_percorrida_escape; // distância reta andada desde o início do escape, em mm (só válido na fase 2)
} TargetVelocities;

// Função principal de tomada de decisão
TargetVelocities process_control_logic(RobotPosition pose, SensorData sensores, double alvo_x, double alvo_y);

#endif