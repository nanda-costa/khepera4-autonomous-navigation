#include <math.h>
#include <stdio.h>
#include "decision.h"

#define TARGET_TOLERANCE  50.0     // Parar a 50mm do alvo (em mm)

// Ganho de correção de rumo: diferença de velocidade de roda por radiano de erro
// de ângulo. Nas mesmas unidades das velocidades de roda (~mm/s), não em rad/s
// físico — evita ter que converter geometria do robô (eixo entre rodas) pra essa
// escala de velocidade "crua" que o firmware usa.
#define GANHO_CORRECAO_RUMO 10.0

// Distância reta mínima de segurança após um desvio antes de começar a corrigir de volta pra reta
#define ESCAPE_MIN_MM       100.0

// Parâmetros da retomada de trajetória: mira num ponto um pouco à frente da
// projeção da posição atual sobre a reta origem->alvo (pursuit com lookahead),
// em vez de mirar direto no alvo final — assim o robô curva de volta pra cima
// da reta original em vez de cortar caminho em diagonal até o alvo
#define LOOKAHEAD_MM         150.0
#define LINE_TOLERANCE_MM    30.0

typedef enum {
    LOGICA_CONDUZIR,
    LOGICA_GIRAR_DESVIO,
    LOGICA_ESCAPE_RETO,
    LOGICA_RETOMAR_TRAJETORIA
} EstadoControle;

static EstadoControle estado_interno = LOGICA_CONDUZIR;
static int dodge_turn_dir = 1; // +1 Esquerda, -1 Direita

// Variáveis de memória para guardar o ponto de início do escape reto
static double escape_x0 = 0.0;
static double escape_y0 = 0.0;
static int iniciou_escape = 0;

// Ponto de partida da missão, define a reta original (origem -> alvo)
static double origem_x = 0.0;
static double origem_y = 0.0;
static int origem_definida = 0;

// Distância perpendicular do ponto (px,py) até a reta (x0,y0)-(x1,y1), em mm
static double distancia_ate_a_reta(double px, double py, double x0, double y0, double x1, double y1) {
    double dx = x1 - x0;
    double dy = y1 - y0;
    double comprimento = hypot(dx, dy);
    if (comprimento < 1e-6) return hypot(px - x0, py - y0);
    return fabs((px - x0) * dy - (py - y0) * dx) / comprimento;
}

// Controlador proporcional de rumo: dado um ponto-alvo (mira_x, mira_y), calcula
// as velocidades de roda pra ir em direção a ele. Usado tanto pelo go-to-goal
// normal (mirando no alvo final) quanto pela retomada de trajetória (mirando
// num ponto sobre a reta original)
static void dirigir_em_direcao_a(RobotPosition pose, double mira_x, double mira_y, double v_base,
                                  double *left_velocity, double *right_velocity) {
    double dx = mira_x - pose.x;
    double dy = mira_y - pose.y;
    double desired_angle = atan2(dy, dx);
    double error = desired_angle - pose.theta;

    while (error >  M_PI) error -= 2.0 * M_PI;
    while (error < -M_PI) error += 2.0 * M_PI;

    double v_robot = v_base * (1.0 - fabs(error) / M_PI);
    double correcao = GANHO_CORRECAO_RUMO * error;

    *left_velocity  = v_robot - correcao;
    *right_velocity = v_robot + correcao;
}

TargetVelocities process_control_logic(RobotPosition pose, SensorData sensores, double alvo_x, double alvo_y) {
    TargetVelocities velocities;

    if (!origem_definida) {
        origem_x = pose.x;
        origem_y = pose.y;
        origem_definida = 1;
    }

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
                velocities.fase_interna   = 0;
                velocities.dist_ate_a_reta = 0.0;
                velocities.dist_percorrida_escape = 0.0;
                break;
            }

            // 2. Mira direto no alvo final (já está sobre/perto da reta original)
            dirigir_em_direcao_a(pose, alvo_x, alvo_y, 35.0,
                                  &velocities.left_velocity, &velocities.right_velocity);
            velocities.estado_atual   = IR_PARA_ALVO; // Estado 0
            velocities.fase_interna   = 0;
            velocities.dist_ate_a_reta = 0.0;
            velocities.dist_percorrida_escape = 0.0;
            break;
        }

        case LOGICA_GIRAR_DESVIO: {
            // Gira parado sobre o próprio eixo até limpar a frente (Igual ao Webots)
            double vel_giro = 30.0;
            velocities.left_velocity  = -dodge_turn_dir * vel_giro;
            velocities.right_velocity =  dodge_turn_dir * vel_giro;
            velocities.estado_atual   = DESVIAR_OBSTACULO; // Estado 1
            velocities.fase_interna   = 1;
            velocities.dist_ate_a_reta = 0.0;
            velocities.dist_percorrida_escape = 0.0;
            break;
        }
        
        case LOGICA_ESCAPE_RETO: {
            // Grava a posição onde a pista limpou
            if (!iniciou_escape) {
                escape_x0 = pose.x;
                escape_y0 = pose.y;
                iniciou_escape = 1;
            }

            // Distância percorrida em linha reta desde o início do escape (em mm)
            double delta_escape = hypot(pose.x - escape_x0, pose.y - escape_y0);

            // Distância perpendicular até a reta original (origem -> alvo), só para telemetria
            double dist_a_linha = distancia_ate_a_reta(pose.x, pose.y, origem_x, origem_y, alvo_x, alvo_y);

            velocities.fase_interna   = 2;
            velocities.dist_ate_a_reta = dist_a_linha;
            velocities.dist_percorrida_escape = delta_escape;
            velocities.left_velocity  = 28.0;
            velocities.right_velocity = 28.0;
            velocities.estado_atual   = IR_PARA_ALVO; // Estado 0

            // Anda reto só a folga mínima de segurança do obstáculo, depois passa
            // pra fase de retomada, que vai curvar de volta pra cima da reta original
            if (delta_escape >= ESCAPE_MIN_MM) {
                estado_interno = LOGICA_RETOMAR_TRAJETORIA;
                iniciou_escape = 0;
            }
            break;
        }

        case LOGICA_RETOMAR_TRAJETORIA: {
            double linha_dx = alvo_x - origem_x;
            double linha_dy = alvo_y - origem_y;
            double linha_len = hypot(linha_dx, linha_dy);
            double ux = (linha_len > 1e-6) ? linha_dx / linha_len : 1.0;
            double uy = (linha_len > 1e-6) ? linha_dy / linha_len : 0.0;

            // Projeta a posição atual sobre a reta origem->alvo e mira num ponto
            // um pouco à frente dessa projeção (lookahead), não no alvo final
            double proj = (pose.x - origem_x) * ux + (pose.y - origem_y) * uy;
            if (proj > linha_len) proj = linha_len;
            double mira_x = origem_x + ux * (proj + LOOKAHEAD_MM);
            double mira_y = origem_y + uy * (proj + LOOKAHEAD_MM);

            double dist_a_linha = distancia_ate_a_reta(pose.x, pose.y, origem_x, origem_y, alvo_x, alvo_y);

            dirigir_em_direcao_a(pose, mira_x, mira_y, 30.0,
                                  &velocities.left_velocity, &velocities.right_velocity);
            velocities.estado_atual   = IR_PARA_ALVO; // Estado 0
            velocities.fase_interna   = 3;
            velocities.dist_ate_a_reta = dist_a_linha;
            velocities.dist_percorrida_escape = 0.0;

            // Já convergiu na reta: volta pro go-to-goal normal, mirando no alvo final
            if (dist_a_linha < LINE_TOLERANCE_MM) {
                estado_interno = LOGICA_CONDUZIR;
            }
            break;
        }
    }

    return velocities;
}