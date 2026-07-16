#include <math.h>
#include <stdio.h>
#include "decision.h"

// Definições físicas do robô (conversão para metros)
#define WHEEL_RADIUS      0.021    // Raio da roda em metros
#define AXLE_LENGTH       0.1054   // Distância entre eixos em metros
#define TARGET_TOLERANCE  50.0     // Parar a 50mm do alvo (em mm)

typedef enum {
    LOGICA_CONDUZIR,
    LOGICA_GIRAR_DESVIO,
    LOGICA_ESCAPE_RETO
} EstadoControle;

static EstadoControle estado_interno = LOGICA_CONDUZIR;
static int dodge_turn_dir = 1; // +1 Esquerda, -1 Direita

// Variáveis de memória para guardar o ponto de início do escape reto
static double escape_x0 = 0.0;
static double escape_y0 = 0.0;
static int iniciou_escape = 0;

TargetVelocities process_control_logic(RobotPosition pose, SensorData sensores, double alvo_x, double alvo_y) {
    TargetVelocities velocities;
    
    // 1. MÁQUINA DE ESTADOS REATIVA (Baseada no código do Webots)
    if (sensores.obstaculo_detectado == 1) {
        if (estado_interno != LOGICA_GIRAR_DESVIO) {
            // Define para onde girar: se o obstáculo está mais forte na esquerda, gira para a direita (-1)
            dodge_turn_dir = (sensores.left_distance > sensores.right_distance) ? -1 : 1;
        }
        estado_interno = LOGICA_GIRAR_DESVIO;
        iniciou_escape = 0; // Reseta a trava do escape reto
    } 
    else {
        // Se a pista limpou e ele estava girando, vai para o escape reto
        if (estado_interno == LOGICA_GIRAR_DESVIO) {
            estado_interno = LOGICA_ESCAPE_RETO;
        }
    }

    // 2. CÁLCULO CINEMÁTICO DOS ESTADOS
    switch (estado_interno) {
        
        case LOGICA_CONDUZIR: {
            // --- CONTROLADOR GO-TO-GOAL (IGUAL AO WEBOTS) ---
            // 1. Distância até o alvo (trabalhando em mm)
            double dx = alvo_x - pose.x;
            double dy = alvo_y - pose.y;
            double dist_to_target = hypot(dx, dy);

            // Se chegou na tolerância, para os motores
            if (dist_to_target < TARGET_TOLERANCE) {
                velocities.left_velocity  = 0.0;
                velocities.right_velocity = 0.0;
                velocities.estado_atual   = PARADO;
                break;
            }

            // 2. Ângulo desejado versus rumo atual
            double desired_angle = atan2(dy, dx);
            double error = desired_angle - pose.theta;

            // Normaliza o erro entre -PI e +PI
            while (error >  M_PI) error -= 2.0 * M_PI;
            while (error < -M_PI) error += 2.0 * M_PI;

            // 3. Velocidade Linear (v) e Angular (w) escaladas para o robô físico
            double v_base = 12.0; // Velocidade base de cruzeiro no chão
            double v_robot = v_base * (1.0 - fabs(error) / M_PI);
            double w_robot = 5.0 * error; // Ganho proporcional do rumo

            // 4. Cinemática Diferencial Inversa (Igual ao Webots, mas na escala física)
            velocities.left_velocity  = v_robot - (w_robot * AXLE_LENGTH / 2.0);
            velocities.right_velocity = v_robot + (w_robot * AXLE_LENGTH / 2.0);
            velocities.estado_atual   = IR_PARA_ALVO; // Estado 0
            break;
        }
        
        case LOGICA_GIRAR_DESVIO: {
            // Gira parado sobre o próprio eixo até limpar a frente (Igual ao Webots)
            double vel_giro = 15.0;
            velocities.left_velocity  = -dodge_turn_dir * vel_giro;
            velocities.right_velocity =  dodge_turn_dir * vel_giro;
            velocities.estado_atual   = DESVIAR_OBSTACULO; // Estado 1
            break;
        }
        
        case LOGICA_ESCAPE_RETO: {
            // Grava a posição onde a pista limpou
            if (!iniciou_escape) {
                escape_x0 = pose.x;
                escape_y0 = pose.y;
                iniciou_escape = 1;
            }

            // Calcula a distância percorrida em linha reta desde o início do escape (em mm)
            double delta_escape = hypot(pose.x - escape_x0, pose.y - escape_y0);

            // Avança reto por 200 mm (0.2 metros, igualzinho ao código do Webots)
            if (delta_escape < 200.0) {
                velocities.left_velocity  = 10.0;
                velocities.right_velocity = 10.0;
                velocities.estado_atual   = IR_PARA_ALVO; // Estado 0
            } else {
                // Afastou-se o suficiente da resma, volta a calcular o rumo ao alvo!
                estado_interno = LOGICA_CONDUZIR;
                iniciou_escape = 0;
            }
            break;
        }
    }

    return velocities;
}