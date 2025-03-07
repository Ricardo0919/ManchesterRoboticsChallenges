#include <micro_ros_arduino.h>
#include <Wire.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>  // Tipo de mensaje para el setpoint y feedback
#include <rclc/rclc.h>
#include <rclc/executor.h>

// --------- PID + Encoder -----------
#define PWM_PIN 14     // Pin de señal PWM para el control
#define EN_A 19        // Encoder canal A
#define EN_B 18        // Encoder canal B
#define IN3_PIN 12     // Dirección 1
#define IN4_PIN 13     // Dirección 2

// Configuración del PWM
const int pwmChannel = 0;   // Canal PWM
const int freq = 980;       // Frecuencia PWM en Hz
const int resolution = 8;   // Resolución de 8 bits (valores de 0 a 255)

// Ganancias del modelo continuo
float kp = 14.87;
float ki = 203.54;
float kd = 0.0;
float T = 0.05;      // Periodo de muestreo en segundos

// Coeficientes PID discretos
float K1, K2, K3;

volatile long pos = 0;      // Acumulador del encoder (debe ser volatile si se modifica en ISR)
long pos_ant = 0;
float vel = 0.0;            // Velocidad medida

// Errores y salida PID
float e[3] = {0, 0, 0};     // e[0]: error actual, e[1]: anterior, e[2]: anterior-2
float u[2] = {0, 0};        // u[0]: salida actual, u[1]: salida anterior

// Setpoint que viene desde ROS
float referencia = 0.0;

// Control de tiempo PID
unsigned long t_anterior = 0;  

// --------- micro-ROS --------------
rcl_subscription_t subscriber;           // Suscriptor para recibir set_point
rcl_publisher_t publisher;              // Publicador para feedback de velocidad
std_msgs__msg__Float32 speed_msg;       // Mensaje para leer set_point
std_msgs__msg__Float32 feedback_msg;    // Mensaje para publicar velocidad

// (1) DECLARAR PUBLICADOR Y MENSAJE PARA PWM NORMALIZADO
rcl_publisher_t pwm_publisher;          // <-- Nuevo publicador
std_msgs__msg__Float32 pwm_msg;         // <-- Nuevo mensaje

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

// -----------------------------------
//      INTERRUPT DEL ENCODER
// -----------------------------------
void IRAM_ATTR int_callback() {
  // Dependiendo del estado de EN_B, incrementa o decrementa la posición
  if (digitalRead(EN_B) == 0) {
    pos++;
  } else {
    pos--;
  }
}

// -----------------------------------
//   CALLBACK para el Subscriber ROS
// -----------------------------------
void motor_speed_callback(const void *msg_in) {
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
  // Actualiza la referencia del PID
  referencia = msg->data;
}

// -----------------------------------
//        Setup micro-ROS + PID
// -----------------------------------
void setup() {
  Serial.begin(115200);

  // micro-ROS
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "pid_controller_node", "", &support);

  // Suscriptor
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "set_point"
  );

  // Publicador para la velocidad (feedback)
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_output"
  );

  // (2) INICIALIZAR PUBLICADOR PWM NORMALIZADO
  rclc_publisher_init_default(
    &pwm_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "motor_pwm"
  );

  // Executor para manejar callback
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &speed_msg,
    &motor_speed_callback,
    ON_NEW_DATA
  );

  // ---- Configuración de pines ----
  pinMode(PWM_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  // Encoder
  pinMode(EN_A, INPUT);
  pinMode(EN_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(EN_A), int_callback, RISING);

  // Configurar PWM en el ESP32
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(PWM_PIN, pwmChannel);

  // ---- Calcular coeficientes del PID discreto ----
  // T es el período de muestreo (0.05 s)
  K1 = kp + (T * ki) + (kd / T);
  K2 = -kp - (2.0 * kd / T);
  K3 = kd / T;

  // Marca de tiempo inicial
  t_anterior = micros();

  Serial.println("micro-ROS PID Controller iniciado.");
}

// -----------------------------------
//              LOOP
// -----------------------------------
void loop() {
  // Procesar eventos ROS (llegada de set_point, etc.)
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); 

  unsigned long t_actual = micros();
  if ( (t_actual - t_anterior) >= (unsigned long)(T * 1000000) ) {
    t_anterior = t_actual;

    // Calcular velocidad a partir del encoder
    vel = (60.0f * 20.0f * (pos - pos_ant) / (12.0f * 34.0f)) * 0.10472f; 
    pos_ant = pos;

    // Calcular error
    e[0] = referencia - vel;

    // PID discreto
    u[0] = (K1 * e[0]) + (K2 * e[1]) + (K3 * e[2]) + u[1];

    // Saturar la salida a -255..255
    if (u[0] > 255)  u[0] = 255;
    if (u[0] < -255) u[0] = -255;

    // Aplicar la salida al motor
    if (u[0] >= 0) {
      digitalWrite(IN3_PIN, HIGH);   // Sentido horario
      digitalWrite(IN4_PIN, LOW);
      ledcWrite(pwmChannel, (int)u[0]);
    } else {
      digitalWrite(IN3_PIN, LOW);
      digitalWrite(IN4_PIN, HIGH);  // Sentido antihorario
      ledcWrite(pwmChannel, (int)(-u[0]));
    }

    // Actualizar historial de errores y salida
    e[2] = e[1];
    e[1] = e[0];
    u[1] = u[0];

    // (3) PUBLICAR FEEDBACK DE VELOCIDAD
    feedback_msg.data = vel;
    rcl_publish(&publisher, &feedback_msg, NULL);

    // (4) PUBLICAR PWM NORMALIZADO: u[0] en [-255, 255] --> dividir por 255
    pwm_msg.data = u[0] / 255.0f; 
    rcl_publish(&pwm_publisher, &pwm_msg, NULL);

    // Debug (opcional)
    // Serial.print("Ref: "); Serial.print(referencia);
    // Serial.print("  Vel: "); Serial.println(vel);
    // Serial.print("  PWM norm: "); Serial.println(pwm_msg.data);
  }

  // Pequeño delay para evitar sobrecarga de CPU (opcional)
  delay(1); 
}
