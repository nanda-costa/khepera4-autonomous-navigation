/*
 * Khepera IV – go to fixed target (1,1) metres, with odometry and obstacle dodging.
 * Starts at (0,0), facing +x.
 * Uses position sensors for wheel odometry (compatible with all Webots versions).
 */

#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/position_sensor.h>   // needed for wheel encoders

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#define MAX_SPEED 47.6                // rad/s
#define WHEEL_RADIUS 0.021            // m (Khepera IV standard)
#define AXLE_LENGTH 0.1054            // m
#define OBSTACLE_THRESHOLD_IR 400.0   // valor puro do IR - reagir se sensor for MAIOR que isso
#define TARGET_TOLERANCE 0.05         // m – stop when this close

#define TARGET_X 1.0
#define TARGET_Y 1.0

#define NUM_ULTRASONIC 5
static const char *ultrasonic_names[NUM_ULTRASONIC] = {
  "left ultrasonic sensor", "front left ultrasonic sensor", "front ultrasonic sensor",
  "front right ultrasonic sensor", "right ultrasonic sensor"
};

#define NUM_INFRARED 12
static const char *infrared_names[NUM_INFRARED] = {
  "rear left infrared sensor", "left infrared sensor", "front left infrared sensor",
  "front infrared sensor", "front right infrared sensor", "right infrared sensor",
  "rear right infrared sensor", "rear infrared sensor",
  "ground left infrared sensor", "ground front left infrared sensor",
  "ground front right infrared sensor", "ground right infrared sensor"
};

static double normalize_angle(double a) {
  while (a > M_PI) a -= 2*M_PI;
  while (a < -M_PI) a += 2*M_PI;
  return a;
}

int main() {
  wb_robot_init();
  int ts = (int)wb_robot_get_basic_time_step();

  // camera
  WbDeviceTag cam = wb_robot_get_device("camera");
  wb_camera_enable(cam, ts);

  // ultrasonic sensors
  WbDeviceTag us[5];
  for (int i = 0; i < 5; ++i) {
    us[i] = wb_robot_get_device(ultrasonic_names[i]);
    wb_distance_sensor_enable(us[i], ts);
  }

  // infrared sensors
  WbDeviceTag ir[12];
  for (int i = 0; i < 12; ++i) {
    ir[i] = wb_robot_get_device(infrared_names[i]);
    wb_distance_sensor_enable(ir[i], ts);
  }

  // LEDs
  WbDeviceTag leds[3] = {
    wb_robot_get_device("front left led"),
    wb_robot_get_device("front right led"),
    wb_robot_get_device("rear led")
  };

  // motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // position sensors
  WbDeviceTag left_encoder = wb_robot_get_device("left wheel sensor");
  WbDeviceTag right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_encoder, ts);
  wb_position_sensor_enable(right_encoder, ts);

  // Initial step to populate sensors
  wb_robot_step(ts);

  // odometry state
  double x = 0.0, y = 0.0, theta = 0.0;
  
  double left_old  = wb_position_sensor_get_value(left_encoder);
  double right_old = wb_position_sensor_get_value(right_encoder);

  int last_sec = 0;
  int dodge_mode = 0;          // 0 = go-to-goal, 1 = dodging
  int dodge_turn_dir = 0;      // +1 left, -1 right

  printf("Going to (%.2f, %.2f) metres.\n", TARGET_X, TARGET_Y);

  while (wb_robot_step(ts) != -1) {
    // --- 1. Update odometry ---
    double left_now  = wb_position_sensor_get_value(left_encoder);
    double right_now = wb_position_sensor_get_value(right_encoder);
    double dl = (left_now  - left_old)  * WHEEL_RADIUS;
    double dr = (right_now - right_old) * WHEEL_RADIUS;
    left_old  = left_now;
    right_old = right_now;

    double ds = (dl + dr) / 2.0;
    double dth = (dr - dl) / AXLE_LENGTH;

    x += ds * cos(theta + dth/2.0);
    y += ds * sin(theta + dth/2.0);
    theta = normalize_angle(theta + dth);

    // --- 2. Print sensors & blink LEDs every second ---
    int sec = (int)wb_robot_get_time();
    if (sec != last_sec) {
      last_sec = sec;
      printf("t=%d s  pose=(%.2f, %.2f, %.2f rad)\n", sec, x, y, theta);
      for (int i = 0; i < 3; ++i)
        wb_led_set(leds[i], 0xFFFFFF & rand());
    }

    // --- 3. Check arrival ---
    double dist_to_target = sqrt((TARGET_X-x)*(TARGET_X-x) + (TARGET_Y-y)*(TARGET_Y-y));
    if (dist_to_target < TARGET_TOLERANCE) {
      wb_motor_set_velocity(left_motor, 0.0);
      wb_motor_set_velocity(right_motor, 0.0);
      printf("Arrived at (%.2f, %.2f)!\n", x, y);
      break;
    }

    // --- 4. Obstacle detection (voltando para infrareds corrigido) ---
    double front_left   = wb_distance_sensor_get_value(ir[2]);
    double front_center = wb_distance_sensor_get_value(ir[3]);
    double front_right  = wb_distance_sensor_get_value(ir[4]);

    // Para o IR: Valores altos significam que o objeto está PERTO
    bool obstacle = (front_left > OBSTACLE_THRESHOLD_IR ||
                     front_center > OBSTACLE_THRESHOLD_IR ||
                     front_right > OBSTACLE_THRESHOLD_IR);

    if (obstacle && !dodge_mode) {
      dodge_mode = 1;
      dodge_turn_dir = (front_left > front_right) ? -1 : +1;   // gira p/ direita se esquerdo bloqueado
      printf("Obstacle! Dodging...\n");
    }

    // --- 5. Control ---
    double ls = 0.0, rs = 0.0;
    if (dodge_mode) {
      // Dodge: turn until front clear, then go forward 0.2 m
      if (front_center > OBSTACLE_THRESHOLD_IR - 100.0) { // Margem de segurança p/ virar
        ls = -dodge_turn_dir * MAX_SPEED * 0.3;
        rs =  dodge_turn_dir * MAX_SPEED * 0.3;
      } else {
        static double dodge_x0, dodge_y0;
        static bool dodge_forward_init = false;
        if (!dodge_forward_init) {
          dodge_x0 = x;
          dodge_y0 = y;
          dodge_forward_init = true;
        }
        double d = sqrt((x-dodge_x0)*(x-dodge_x0) + (y-dodge_y0)*(y-dodge_y0));
        if (d < 0.2) {
          ls = rs = MAX_SPEED * 0.3;
        } else {
          dodge_mode = 0;
          dodge_forward_init = false;
          printf("Dodge finished.\n");
        }
      }
    } else {
      // Go-to-goal controller
      double desired_angle = atan2(TARGET_Y - y, TARGET_X - x);
      double error = normalize_angle(desired_angle - theta);

      // NOVO CÁLCULO CINEMÁTICO: Trabalha em unidades corretas de Física (m/s e rad/s)
      double max_v_robot = MAX_SPEED * WHEEL_RADIUS; // Velocidade máxima teórica em m/s (~1.0)
      
      // Velocidade Linear (v_robot) em m/s e Velocidade Angular (w_robot) em rad/s
      double v_robot = max_v_robot * 0.3 * (1.0 - fabs(error)/M_PI); 
      double w_robot = 4.0 * error; // ganho proporcional forte

      // Calculando a velocidade da borda de cada roda (m/s)
      double v_left  = v_robot - (w_robot * AXLE_LENGTH / 2.0);
      double v_right = v_robot + (w_robot * AXLE_LENGTH / 2.0);

      // Convertendo a velocidade da roda para velocidade rotacional do motor (rad/s)
      ls = v_left / WHEEL_RADIUS;
      rs = v_right / WHEEL_RADIUS;

      // Saturate
      if (ls > MAX_SPEED) ls = MAX_SPEED;
      else if (ls < -MAX_SPEED) ls = -MAX_SPEED;
      if (rs > MAX_SPEED) rs = MAX_SPEED;
      else if (rs < -MAX_SPEED) rs = -MAX_SPEED;
    }

    wb_motor_set_velocity(left_motor, ls);
    wb_motor_set_velocity(right_motor, rs);
  }

  wb_robot_cleanup();
  return 0;
}