import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import SetProcessBool

# Clase que implementa un controlador PID en ROS2
class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Declaración de parámetros ajustables del controlador PID
        self.declare_parameter('Kp', 0.005)  # Ganancia proporcional
        self.declare_parameter('Ki', 2.8)    # Ganancia integral
        self.declare_parameter('Kd', 0.0)    # Ganancia derivativa
        self.declare_parameter('sample_time', 0.01)  # Tiempo de muestreo

        # Obtener valores iniciales de los parámetros
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.sample_time = self.get_parameter('sample_time').value

        # Inicialización de variables del controlador
        self.set_point = 0.0  # Valor deseado de referencia
        self.motor_speed = 0.0  # Velocidad actual del motor
        self.control_signal = 0.0  # Señal de control calculada
        self.integral = 0.0  # Componente integral del PID
        self.previous_error = 0.0  # Error previo para el término derivativo
        self.system_running = False  # Indica si la simulación está activa

        # Crear cliente de servicio para iniciar el motor
        self.cli = self.create_client(SetProcessBool, 'StartMotor')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Esperando disponibilidad del servicio StartMotor...")
        self.send_request(True)  # Enviar solicitud para iniciar la simulación

        # Publicador de la señal de control PID al motor
        self.control_pub = self.create_publisher(Float32, 'motor_input_u', 10)

        # Suscriptores para recibir el set point y la velocidad del motor
        self.create_subscription(Float32, 'set_point', self.set_point_callback, 10)
        self.create_subscription(Float32, 'motor_speed_y', self.motor_speed_callback, 10)

        # Suscriptor para el control de la simulación (encender/apagar)
        self.create_subscription(Bool, 'simulation_control', self.control_callback, 10)

        # Temporizador para ejecutar el bucle de control periódicamente
        self.timer = self.create_timer(self.sample_time, self.control_loop)

        # Callback para actualización de parámetros dinámicamente
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info('Controller Node Iniciado 🚀')

    def control_callback(self, msg):
        """Recibe el estado de la simulación (encendida o apagada)."""
        self.system_running = msg.data

    def set_point_callback(self, msg):
        """Recibe el valor de referencia (set point) del controlador."""
        self.set_point = msg.data

    def motor_speed_callback(self, msg):
        """Recibe la velocidad actual del motor."""
        self.motor_speed = msg.data

    def control_loop(self):
        """Ejecuta el cálculo del control PID y publica la señal de control."""
        if not self.system_running:
            return  # No ejecutar el control si la simulación está apagada

        error = self.set_point - self.motor_speed  # Calcular el error actual

        # Cálculo de los términos del controlador PID
        proportional = self.Kp * error  # Componente proporcional
        self.integral += error * self.sample_time  # Acumulación del término integral
        integral = self.Ki * self.integral  # Componente integral
        derivative = self.Kd * (error - self.previous_error) / self.sample_time  # Componente derivativo

        # Calcular la señal de control final
        self.control_signal = proportional + integral + derivative
        self.previous_error = error  # Guardar el error actual para la próxima iteración

        # Publicar la señal de control
        control_msg = Float32()
        control_msg.data = self.control_signal
        self.control_pub.publish(control_msg)

    def parameters_callback(self, params):
        """Callback para modificar parámetros del controlador en tiempo de ejecución."""
        for param in params:
            if param.name == "Kp":
                if param.value < 0.0:
                    self.get_logger().warn("Kp no puede ser negativo")
                    return SetParametersResult(successful=False, reason="Kp no puede ser negativo")
                self.Kp = param.value
                self.get_logger().info(f"Kp actualizado a {self.Kp}")

            if param.name == "Ki":
                if param.value < 0.0:
                    self.get_logger().warn("Ki no puede ser negativo")
                    return SetParametersResult(successful=False, reason="Ki no puede ser negativo")
                self.Ki = param.value
                self.get_logger().info(f"Ki actualizado a {self.Ki}")

            if param.name == "Kd":
                if param.value < 0.0:
                    self.get_logger().warn("Kd no puede ser negativo")
                    return SetParametersResult(successful=False, reason="Kd no puede ser negativo")
                self.Kd = param.value
                self.get_logger().info(f"Kd actualizado a {self.Kd}")

            if param.name == "sample_time":
                if param.value <= 0.0:
                    self.get_logger().warn("sample_time debe ser mayor a 0")
                    return SetParametersResult(successful=False, reason="sample_time debe ser mayor a 0")
                self.sample_time = param.value
                self.get_logger().info(f"sample_time actualizado a {self.sample_time}")

        return SetParametersResult(successful=True)
    
    def send_request(self, enable: bool):
        """Envía una solicitud al servicio para iniciar o detener la simulación."""
        request = SetProcessBool.Request()
        request.enable = enable
        future = self.cli.call_async(request)  # Llamada asíncrona al servicio
        future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        """Procesa la respuesta del servicio StartMotor."""
        try:
            response = future.result()
            if response.success:
                self.system_running = True  # Activar la simulación
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.system_running = False  # Detener la simulación
                self.get_logger().warn(f'Failure: {response.message}')
        except Exception as e:
            self.system_running = False
            self.get_logger().error(f'Fallo en la llamada al servicio: {e}')

# Función principal que inicia el nodo de ROS2
def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)  # Mantener el nodo en ejecución
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
