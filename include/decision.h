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
} TargetVelocities;

// Função principal de tomada de decisão
TargetVelocities process_control_logic(RobotPosition pose, SensorData sensores, double alvo_x, double alvo_y);

#endif