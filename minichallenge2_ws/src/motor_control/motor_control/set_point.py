# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import SetProcessBool

# Class Definition
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        # Declarar parámetros dinámicos
        self.declare_parameter('amplitude', 2.0)
        self.declare_parameter('omega', 1.0)
        self.declare_parameter('signal_type', 'sine')  # 'sine', 'square', 'step'
        self.declare_parameter('step_value', 2.0)  # Valor del escalón
        
        # Obtener valores iniciales
        self.amplitude = self.get_parameter('amplitude').value
        self.omega = self.get_parameter('omega').value
        self.signal_type = self.get_parameter('signal_type').value
        self.step_value = self.get_parameter('step_value').value
        
        self.timer_period = 0.1  # segundos

        # Crear un publicador y temporizador para la señal
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        # Agregar un suscriptor al tópico de control de la simulación
        self.create_subscription(Bool, 'simulation_control', self.control_callback, 10)
        
        # Variables internas
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()
        self.system_running = False

        #Crear un cliente de servicio para /StartMotor
        self.cli = self.create_client(SetProcessBool, 'StartMotor')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.send_request(True)

        # Callback para actualizar parámetros dinámicos
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info("SetPoint Node Started 🚀")

    # Callback para el tópico de control
    def control_callback(self, msg):
        self.system_running = msg.data

    def timer_cb(self):
        #Mientras la simulación no esté corriendo, detener el procesamiento

        if not self.system_running:
            return # Detener el procesamiento si la simulación no está corriendo

        """Genera y publica la señal de acuerdo con el tipo seleccionado."""
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.signal_type == 'sine':
            self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)
        elif self.signal_type == 'square':
            self.signal_msg.data = self.amplitude if np.sin(self.omega * elapsed_time) >= 0 else -self.amplitude
        elif self.signal_type == 'step':
            self.signal_msg.data = self.step_value
        else:
            self.get_logger().warn(f"Tipo de señal desconocido: {self.signal_type}")
            return

        # Publicar la señal
        self.signal_publisher.publish(self.signal_msg)

    def parameters_callback(self, params):
        """Callback para actualizar los parámetros en tiempo de ejecución."""
        for param in params:
            if param.name == "amplitude":
                self.amplitude = param.value
                self.get_logger().info(f"Amplitud actualizada a {self.amplitude}")

            if param.name == "omega":
                self.omega = param.value
                self.get_logger().info(f"Frecuencia omega actualizada a {self.omega}")

            if param.name == "signal_type":
                self.signal_type = param.value
                self.get_logger().info(f"Tipo de señal actualizado a {self.signal_type}")

            if param.name == "step_value":
                self.step_value = param.value
                self.get_logger().info(f"Valor del escalón actualizado a {self.step_value}")

        return SetParametersResult(successful=True)
    
    def send_request(self, enable: bool):
        """Envía una solicitud al servicio para iniciar o detener la simulación."""
        request = SetProcessBool.Request()
        request.enable = enable
        #Send a request to start or stop the simulation
        future = self.cli.call_async(request)
        future.add_done_callback(self.response_callback)
    
    def response_callback(self, future):
        """Process the service response."""
        try:
            response = future.result()
            if response.success:
                self.system_running = True
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.simulation_running = False
                self.get_logger().warn(f'Failure: {response.message}')
        except Exception as e:
            self.simulation_running = False
            self.get_logger().error(f'Service call failed: {e}')

# Main
def main(args=None):
    rclpy.init(args=args)
    set_point = SetPointPublisher()

    try:
        rclpy.spin(set_point)
    except KeyboardInterrupt:
        pass
    finally:
        set_point.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
