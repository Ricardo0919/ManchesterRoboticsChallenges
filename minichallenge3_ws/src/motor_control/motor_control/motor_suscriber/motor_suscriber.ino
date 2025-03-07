#include <micro_ros_arduino.h>
#include <Wire.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>  // Mensaje para recibir velocidad (-1 a 1)
#include <std_msgs/msg/string.h>   // Mensaje para enviar feedback
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define PWM_PIN 14  // Pin de señal PWM para el control de velocidad
#define IN3_PIN 13  // Pin para la dirección del motor
#define IN4_PIN 12  // Pin para la dirección opuesta

// Declaración de los objetos de ROS 2
rcl_subscription_t subscriber; // Suscriptor para recibir comandos de velocidad
rcl_publisher_t publisher;  // Publicador para enviar feedback del motor
std_msgs__msg__Float32 speed_msg; // Mensaje para almacenar la velocidad recibida
std_msgs__msg__String feedback_msg; // Mensaje para enviar feedback del motor
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

// Configuración del PWM
const int pwmChannel = 0; // Canal PWM
const int freq = 980; // Frecuencia PWM en Hz
const int resolution = 8;  // Resolución de 8 bits (valores de 0 a 255)

// Callback para recibir y procesar la velocidad del motor
void motor_speed_callback(const void *msg_in) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
    float speed = msg->data;

    // Limitar la velocidad entre -1 y 1
    speed = constrain(speed, -1.0, 1.0);

   // Convertir la velocidad (-1 a 1) en señal PWM (0 a 255)
    int pwmValue = abs(speed) * 255;

    // Control de dirección del motor según el signo de la velocidad
    if (speed > 0) {
        digitalWrite(IN3_PIN, HIGH);
        digitalWrite(IN4_PIN, LOW);
    } else if (speed < 0) {
        digitalWrite(IN3_PIN, LOW);
        digitalWrite(IN4_PIN, HIGH);
    } else {
        digitalWrite(IN3_PIN, LOW);
        digitalWrite(IN4_PIN, LOW);
    }

    // Aplicar la señal PWM al motor
    ledcWrite(pwmChannel, pwmValue);

    // Crear mensaje de feedback con la información del PWM
    char feedback_str[50];
    snprintf(feedback_str, sizeof(feedback_str), "PWM: %.2f | Speed: %d", speed, pwmValue);

    // Configurar el mensaje de feedback antes de publicarlo
    feedback_msg.data.data = feedback_str;
    feedback_msg.data.size = strlen(feedback_str);
    feedback_msg.data.capacity = sizeof(feedback_str);

    // Publicar feedback en ROS 2
    rcl_publish(&publisher, &feedback_msg, NULL);
}

void setup() {
    Serial.begin(115200); // Inicializar comunicación serial para depuración
    set_microros_transports(); // Configurar transporte para micro-ROS

    // Configurar pines de control del motor como salida
    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
    pinMode(PWM_PIN, OUTPUT);

    // Configurar PWM en el canal definido
    ledcSetup(pwmChannel, freq, resolution);
    ledcAttachPin(PWM_PIN, pwmChannel);

    // Inicializar ROS con asignador de memoria por defecto
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "motor", "", &support);

    // Configurar publicador para enviar feedback
    rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "motor_feedback");

    // Configurar suscriptor para recibir velocidad del motor
    rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "cmd_pwm");

    // Inicializar el ejecutor para gestionar las comunicaciones de ROS
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &speed_msg, &motor_speed_callback, ON_NEW_DATA);
}

void loop() {
    delay(10); // Pequeño retardo para evitar uso excesivo de CPU
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)); // Procesar eventos de ROS 
}