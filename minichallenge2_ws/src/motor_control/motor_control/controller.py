import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import SetProcessBool

# Class Definition
class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # Declarar parámetros ajustables
        self.declare_parameter('Kp', 0.005)  # Ganancia Proporcional
        self.declare_parameter('Ki', 2.8)  # Ganancia Integral
        self.declare_parameter('Kd', 0.0)  # Ganancia Derivativa
        self.declare_parameter('sample_time', 0.01)  # Tiempo de muestreo

        # Obtener valores iniciales de parámetros
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value
        self.sample_time = self.get_parameter('sample_time').value

        # Inicializar variables
        self.set_point = 0.0
        self.motor_speed = 0.0
        self.control_signal = 0.0
        self.integral = 0.0
        self.previous_error = 0.0
        self.system_running = False

        #Crear un cliente de servicio para /StartMotor
        self.cli = self.create_client(SetProcessBool, 'StartMotor')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.send_request(True)

        # Publicador de la señal de control
        self.control_pub = self.create_publisher(Float32, 'motor_input_u', 10)

        # Suscriptores a los tópicos de referencia y salida del motor
        self.create_subscription(Float32, 'set_point', self.set_point_callback, 10)
        self.create_subscription(Float32, 'motor_speed_y', self.motor_speed_callback, 10)

        # Agregar un suscriptor al tópico de control de la simulación
        self.create_subscription(Bool, 'simulation_control', self.control_callback, 10)

        # Timer para ejecutar el controlador periódicamente
        self.timer = self.create_timer(self.sample_time, self.control_loop)

        # Callback para actualización de parámetros
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info('Controller Node Started 🚀')

    # Callback para el tópico de control
    def control_callback(self, msg):
        self.system_running = msg.data

    def set_point_callback(self, msg):
        """Callback para recibir el valor de referencia (set_point)."""
        self.set_point = msg.data

    def motor_speed_callback(self, msg):
        """Callback para recibir la velocidad del motor."""
        self.motor_speed = msg.data

    def control_loop(self):
        """Bucle principal del controlador."""
        
        # Mientras la simulación no esté corriendo, detener el procesamiento
        if not self.system_running:
            return  # No ejecutar el controlador si la simulación no está corriendo

        error = self.set_point - self.motor_speed  # Cálculo del error

        # Término Proporcional
        proportional = self.Kp * error

        # Término Integral
        self.integral += error * self.sample_time
        integral = self.Ki * self.integral

        # Término Derivativo
        derivative = self.Kd * (error - self.previous_error) / self.sample_time

        # Señal de control
        self.control_signal = proportional + integral + derivative

        # Guardar el error actual para el próximo cálculo de la derivada
        self.previous_error = error

        # Publicar la señal de control
        control_msg = Float32()
        control_msg.data = self.control_signal
        self.control_pub.publish(control_msg)


    def parameters_callback(self, params):
        """Callback para modificar parámetros en tiempo de ejecución."""
        for param in params:
            if param.name == "Kp":
                if param.value < 0.0:
                    self.get_logger().warn("Kp no puede ser negativo")
                    return SetParametersResult(successful=False, reason="Kp no puede ser negativo")
                else:
                    self.Kp = param.value
                    self.get_logger().info(f"Kp actualizado a {self.Kp}")

            if param.name == "Ki":
                if param.value < 0.0:
                    self.get_logger().warn("Ki no puede ser negativo")
                    return SetParametersResult(successful=False, reason="Ki no puede ser negativo")
                else:
                    self.Ki = param.value
                    self.get_logger().info(f"Ki actualizado a {self.Ki}")

            if param.name == "Kd":
                if param.value < 0.0:
                    self.get_logger().warn("Kd no puede ser negativo")
                    return SetParametersResult(successful=False, reason="Kd no puede ser negativo")
                else:
                    self.Kd = param.value
                    self.get_logger().info(f"Kd actualizado a {self.Kd}")

            if param.name == "sample_time":
                if param.value <= 0.0:
                    self.get_logger().warn("sample_time debe ser mayor a 0")
                    return SetParametersResult(successful=False, reason="sample_time debe ser mayor a 0")
                else:
                    self.sample_time = param.value
                    self.get_logger().info(f"sample_time actualizado a {self.sample_time}")

        return SetParametersResult(successful=True)
    
    def send_request(self, enable: bool):
        """Envía una solicitud al servicio para iniciar o detener la simulación."""
        request = SetProcessBool.Request()
        request.enable = enable
        #Send a request to start or stop the simulation
        future = self.cli.call_async(request)
        future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        """Procesar la respuesta del servicio."""
        try:
            response = future.result()
            if response.success:
                self.system_running = True  # Se activa la simulación
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.system_running = False  # Se detiene la simulación
                self.get_logger().warn(f'Failure: {response.message}')
        except Exception as e:
            self.system_running = False
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()