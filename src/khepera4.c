#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <math.h>
#include <stdio.h>
#include <stdbool.h>

// Definicoes e Constantes do Khepera IV
#define TIME_STEP 32
#define MAX_SPEED 47.6       // Velocidade maxima do motor em rad/s
#define WHEEL_RADIUS 0.021   // Raio da roda em metros (21mm)
#define AXLE_LENGTH 0.1052   // Distancia entre as rodas em metros (~105mm)
#define NUM_US_SENSORS 5     // Quantidade de sensores ultrasonicos

// Distancia de seguranca para ativar o desvio (em metros)
#define DISTANCIA_SEGURANCA 0.25

// Estados do robo
typedef enum {
    IR_PARA_ALVO,
    DESVIAR_OBSTACULO,
    PARADO
} EstadoRobo;

// Variaveis globais de Odometria
double x_atual = 0.0;
double y_atual = 0.0;
double theta_atual = 0.0; // Angulo em radianos
double last_pos_l = 0.0;
double last_pos_r = 0.0;
bool primeira_leitura = true;

// Funcao para normalizar angulos entre -PI e PI
double normalizar_angulo(double angulo) {
    while (angulo > M_PI) angulo -= 2.0 * M_PI;
    while (angulo < -M_PI) angulo += 2.0 * M_PI;
    return angulo;
}

// Funcao para atualizar a odometria
void atualizar_odometria(WbDeviceTag pos_sensor_l, WbDeviceTag pos_sensor_r) {
    double pos_l = wb_position_sensor_get_value(pos_sensor_l);
    double pos_r = wb_position_sensor_get_value(pos_sensor_r);

    if (primeira_leitura) {
        last_pos_l = pos_l;
        last_pos_r = pos_r;
        primeira_leitura = false;
        return;
    }

    // Calcula a diferenca de rotacao das rodas
    double dl = (pos_l - last_pos_l) * WHEEL_RADIUS;
    double dr = (pos_r - last_pos_r) * WHEEL_RADIUS;

    // Distancia linear percorrida e variacao de angulo
    double delta_distancia = (dl + dr) / 2.0;
    double delta_angulo = (dr - dl) / AXLE_LENGTH;

    // Atualiza a pose (x, y, theta) do robo
    x_atual += delta_distancia * cos(theta_atual + delta_angulo / 2.0);
    y_atual += delta_distancia * sin(theta_atual + delta_angulo / 2.0);
    theta_atual += delta_angulo;
    theta_atual = normalizar_angulo(theta_atual);

    // Atualiza leituras anteriores
    last_pos_l = pos_l;
    last_pos_r = pos_r;
}

int main() {
    wb_robot_init();

    // Inicializacao dos Motores
    WbDeviceTag motor_esquerdo = wb_robot_get_device("left wheel motor");
    WbDeviceTag motor_direito = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(motor_esquerdo, INFINITY); // Modo de velocidade
    wb_motor_set_position(motor_direito, INFINITY);
    wb_motor_set_velocity(motor_esquerdo, 0.0);
    wb_motor_set_velocity(motor_direito, 0.0);

    // Inicializacao dos Sensores de Posicao (Encoders para Odometria)
    WbDeviceTag pos_sensor_l = wb_robot_get_device("left wheel sensor");
    WbDeviceTag pos_sensor_r = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable(pos_sensor_l, TIME_STEP);
    wb_position_sensor_enable(pos_sensor_r, TIME_STEP);

    // Inicializacao dos Sensores Ultrassonicos
    // Khepera IV possui 5 US (us0: Esq, us1: Front-Esq, us2: Frontal, us3: Front-Dir, us4: Dir)
    // NOTA: Confirme se os nomes batem com a sua Scene Tree.
    char *us_nomes[NUM_US_SENSORS] = {"us0", "us1", "us2", "us3", "us4"};
    WbDeviceTag us_sensors[NUM_US_SENSORS];
    
    for (int i = 0; i < NUM_US_SENSORS; i++) {
        us_sensors[i] = wb_robot_get_device(us_nomes[i]);
        if (us_sensors[i] != 0) {
            wb_distance_sensor_enable(us_sensors[i], TIME_STEP);
        } else {
            printf("Aviso: Sensor %s nao encontrado.\n", us_nomes[i]);
        }
    }

    // Definicao do Destino
    double alvo_x = 1.0;
    double alvo_y = 1.0;
    
    EstadoRobo estado_atual = IR_PARA_ALVO;
    
    printf("Iniciando missao: Indo de (0,0) para (%.1f, %.1f)\n", alvo_x, alvo_y);

    // Loop Principal
    while (wb_robot_step(TIME_STEP) != -1) {
        atualizar_odometria(pos_sensor_l, pos_sensor_r);

        // Leitura dos sensores ultrassonicos
        double leituras_us[NUM_US_SENSORS];
        bool obstaculo_detectado = false;
        double forca_desvio = 0.0; // Positivo vira para direita, negativo para esquerda

        for (int i = 0; i < NUM_US_SENSORS; i++) {
            if (us_sensors[i] != 0) {
                leituras_us[i] = wb_distance_sensor_get_value(us_sensors[i]);
                // Se a distancia medida for menor que o limite (0.25m)
                if (leituras_us[i] < DISTANCIA_SEGURANCA) {
                    obstaculo_detectado = true;
                    // Se o obstaculo esta na esquerda (us0, us1), viramos para direita
                    if (i == 0 || i == 1) forca_desvio += 1.0;
                    // Se o obstaculo esta na direita (us3, us4), viramos para esquerda
                    if (i == 3 || i == 4) forca_desvio -= 1.0;
                    // Se estiver no meio exato (us2), forca o giro pra um lado padrao
                    if (i == 2) forca_desvio -= 1.5; 
                }
            }
        }

        // Variaveis de controle de velocidade das rodas
        double v_esq = 0.0;
        double v_dir = 0.0;

        // Calculo da distancia ate o alvo
        double erro_x = alvo_x - x_atual;
        double erro_y = alvo_y - y_atual;
        double distancia_alvo = sqrt(erro_x * erro_x + erro_y * erro_y);

        // Verifica se chegou ao destino (margem de 5cm)
        if (distancia_alvo < 0.05) {
            estado_atual = PARADO;
        } else if (obstaculo_detectado) {
            estado_atual = DESVIAR_OBSTACULO;
        } else {
            estado_atual = IR_PARA_ALVO;
        }

        // Execucao da Maquina de Estados
        switch (estado_atual) {
            case IR_PARA_ALVO: {
                // Controlador Proporcional para seguir ao alvo
                double angulo_desejado = atan2(erro_y, erro_x);
                double erro_angulo = normalizar_angulo(angulo_desejado - theta_atual);
                
                // Ganhos do controlador
                double k_linear = 2.0;
                double k_angular = 4.0;
                
                // Se o erro de angulo for muito grande, apenas gira no eixo
                if (fabs(erro_angulo) > 0.4) {
                    v_esq = -k_angular * erro_angulo;
                    v_dir =  k_angular * erro_angulo;
                } else {
                    // Anda pra frente corrigindo o angulo
                    double v_linear = k_linear * distancia_alvo;
                    if (v_linear > MAX_SPEED / 2.0) v_linear = MAX_SPEED / 2.0; // Limita a velocidade
                    
                    double v_angular = k_angular * erro_angulo;
                    
                    v_esq = v_linear - v_angular;
                    v_dir = v_linear + v_angular;
                }
                break;
            }
            case DESVIAR_OBSTACULO: {
                // Rotaciona fugindo do obstaculo (In-place rotation)
                double vel_rotacao = 10.0; 
                if (forca_desvio >= 0) { // Obstaculo na esquerda ou centro
                    v_esq = vel_rotacao;
                    v_dir = -vel_rotacao;
                } else { // Obstaculo na direita
                    v_esq = -vel_rotacao;
                    v_dir = vel_rotacao;
                }
                break;
            }
            case PARADO:
                v_esq = 0.0;
                v_dir = 0.0;
                printf("Destino (1,1) alcancado!\n");
                break;
        }

        // Limitando a velocidade das rodas para nao ultrapassar o maximo do motor
        if (v_esq > MAX_SPEED) v_esq = MAX_SPEED;
        if (v_esq < -MAX_SPEED) v_esq = -MAX_SPEED;
        if (v_dir > MAX_SPEED) v_dir = MAX_SPEED;
        if (v_dir < -MAX_SPEED) v_dir = -MAX_SPEED;

        // Imprime a posicao a cada ~1 segundo para acompanhamento
        static int contador_print = 0;
        if (contador_print++ % 30 == 0 && estado_atual != PARADO) {
            printf("Posicao: X=%.2f m, Y=%.2f m, Angulo=%.2f rad | Dist: %.2f m\n", 
                   x_atual, y_atual, theta_atual, distancia_alvo);
        }

        // Aplica as velocidades nos motores
        wb_motor_set_velocity(motor_esquerdo, v_esq);
        wb_motor_set_velocity(motor_direito, v_dir);
        
        // Finaliza o laco caso atinja o destino
        if (estado_atual == PARADO) break;
    }

    wb_robot_cleanup();
    return 0;
}