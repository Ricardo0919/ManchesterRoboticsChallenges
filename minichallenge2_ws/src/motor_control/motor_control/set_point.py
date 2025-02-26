# Imports
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import SetProcessBool

# Definición de la clase que publica el set point
class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        # Declaración de parámetros dinámicos para controlar la señal
        self.declare_parameter('amplitude', 2.0)  # Amplitud de la señal
        self.declare_parameter('omega', 1.0)  # Frecuencia angular de la señal
        self.declare_parameter('signal_type', 'sine')  # Tipo de señal ('sine', 'square', 'step')
        self.declare_parameter('step_value', 2.0)  # Valor del escalón para señales tipo 'step'
        
        # Obtener valores iniciales de los parámetros declarados
        self.amplitude = self.get_parameter('amplitude').value
        self.omega = self.get_parameter('omega').value
        self.signal_type = self.get_parameter('signal_type').value
        self.step_value = self.get_parameter('step_value').value
        
        self.timer_period = 0.1  # Periodo del temporizador para publicar la señal cada 0.1 segundos

        # Crear un publicador para la señal de set point
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)
        # Crear un temporizador que llamará al callback cada cierto periodo
        self.timer = self.create_timer(self.timer_period, self.timer_cb)

        # Crear un suscriptor para controlar el estado de la simulación
        self.create_subscription(Bool, 'simulation_control', self.control_callback, 10)
        
        # Variables internas
        self.signal_msg = Float32()  # Mensaje de señal a publicar
        self.start_time = self.get_clock().now()  # Tiempo de inicio de la simulación
        self.system_running = False  # Estado de la simulación (inicialmente detenido)

        # Crear un cliente de servicio para iniciar o detener el motor
        self.cli = self.create_client(SetProcessBool, 'StartMotor')
        # Esperar hasta que el servicio esté disponible
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.send_request(True)  # Enviar solicitud para iniciar el motor

        # Callback para actualizar los parámetros cuando se cambian
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Mensaje de inicio de nodo
        self.get_logger().info("SetPoint Node Started 🚀")

    # Callback para el control de la simulación
    def control_callback(self, msg):
        self.system_running = msg.data  # Actualiza el estado de la simulación

    def timer_cb(self):
        # Si la simulación no está corriendo, detiene el procesamiento
        if not self.system_running:
            return  # Detener el procesamiento si la simulación no está corriendo

        """Genera y publica la señal de acuerdo con el tipo seleccionado."""
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9  # Tiempo transcurrido en segundos

        # Generación de señales de diferentes tipos
        if self.signal_type == 'sine':
            self.signal_msg.data = self.amplitude * np.sin(self.omega * elapsed_time)  # Señal seno
        elif self.signal_type == 'square':
            self.signal_msg.data = self.amplitude if np.sin(self.omega * elapsed_time) >= 0 else -self.amplitude  # Señal cuadrada
        elif self.signal_type == 'step':
            self.signal_msg.data = self.step_value  # Señal escalón
        else:
            # Advertencia si el tipo de señal no es válido
            self.get_logger().warn(f"Tipo de señal desconocido: {self.signal_type}")
            return

        # Publicar la señal generada en el tópico 'set_point'
        self.signal_publisher.publish(self.signal_msg)

    def parameters_callback(self, params):
        """Callback para actualizar los parámetros en tiempo de ejecución."""
        for param in params:
            # Actualiza los parámetros en tiempo de ejecución y registra los cambios
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

        return SetParametersResult(successful=True)  # Indicamos que el cambio fue exitoso
    
    def send_request(self, enable: bool):
        """Envía una solicitud al servicio para iniciar o detener la simulación."""
        request = SetProcessBool.Request()
        request.enable = enable  # Establecer el estado deseado (iniciar o detener)
        # Llamar al servicio de forma asíncrona
        future = self.cli.call_async(request)
        future.add_done_callback(self.response_callback)  # Procesar la respuesta del servicio cuando esté disponible
    
    def response_callback(self, future):
        """Procesar la respuesta del servicio de inicio/detención del motor."""
        try:
            response = future.result()  # Obtener la respuesta del servicio
            if response.success:
                self.system_running = True  # El motor ha sido iniciado correctamente
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.system_running = False  # El motor no se ha iniciado correctamente
                self.get_logger().warn(f'Failure: {response.message}')
        except Exception as e:
            self.system_running = False  # En caso de error, detener la simulación
            self.get_logger().error(f'Service call failed: {e}')

# Main: Función principal que inicializa el nodo
def main(args=None):
    rclpy.init(args=args)  # Inicializar rclpy
    set_point = SetPointPublisher()  # Crear el nodo para publicar el set point

    try:
        rclpy.spin(set_point)  # Mantener el nodo en ejecución
    except KeyboardInterrupt:
        pass  # Permitir interrumpir con CTRL+C
    finally:
        set_point.destroy_node()  # Destruir el nodo al finalizar
        rclpy.try_shutdown()  # Apagar rclpy

if __name__ == '__main__':
    main()  # Ejecutar la función principal al iniciar el script
