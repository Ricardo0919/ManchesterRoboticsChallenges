#include <micro_ros_arduino.h>
#include <Wire.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/float32.h>  // Para recibir velocidad (-1 a 1)
#include <std_msgs/msg/string.h>   // Para enviar feedback
#include <rclc/rclc.h>
#include <rclc/executor.h>

#define PWM_PIN 14  // Pin PWM
#define IN3_PIN 13  // Dirección del motor
#define IN4_PIN 12  // Dirección opuesta

rcl_subscription_t subscriber;
rcl_publisher_t publisher;  // Nuevo publisher
std_msgs__msg__Float32 speed_msg;
std_msgs__msg__String feedback_msg;
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

const int pwmChannel = 0;
const int freq = 980;
const int resolution = 8;  // 8 bits = valores de 0 a 255

// Función callback: recibe velocidad entre -1 y 1
void motor_speed_callback(const void *msg_in) {
    const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msg_in;
    float speed = msg->data;

    // Limitar valores entre -1 y 1
    speed = constrain(speed, -1.0, 1.0);

    // Convertir -1 a 1 en PWM (0 a 255)
    int pwmValue = abs(speed) * 255;

    // Controlar dirección
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

    // Aplicar PWM
    ledcWrite(pwmChannel, pwmValue);

    // Crear mensaje de feedback correctamente
    char feedback_str[50];
    snprintf(feedback_str, sizeof(feedback_str), "Vel: %.2f | PWM: %d", speed, pwmValue);

    feedback_msg.data.data = feedback_str;  // Asignar string
    feedback_msg.data.size = strlen(feedback_str);
    feedback_msg.data.capacity = sizeof(feedback_str);

    rcl_publish(&publisher, &feedback_msg, NULL);

}

void setup() {
    Serial.begin(115200);
    set_microros_transports();

    pinMode(IN3_PIN, OUTPUT);
    pinMode(IN4_PIN, OUTPUT);
    pinMode(PWM_PIN, OUTPUT);

    ledcSetup(pwmChannel, freq, resolution);
    ledcAttachPin(PWM_PIN, pwmChannel);

    allocator = rcl_get_default_allocator();

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "motor_node", "", &support);

    // Inicializar Publisher
    rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "motor_feedback");

    // Inicializar Subscriber
    rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "motor_speed");

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &speed_msg, &motor_speed_callback, ON_NEW_DATA);
}

void loop() {
    delay(10);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
