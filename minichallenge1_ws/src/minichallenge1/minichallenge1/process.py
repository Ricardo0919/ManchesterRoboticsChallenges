import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class SignalProcessor(Node):
    def __init__(self):
        super().__init__('process')
        
        # Suscriptores a los temas "/signal" y "/time"
        self.signal_subscription = self.create_subscription(Float32, '/signal', self.signal_callback, 10)
        self.time_subscription = self.create_subscription(Float32, '/time', self.time_callback, 10)
        
        # Publicador al tema "/proc_signal"
        self.proc_signal_publisher = self.create_publisher(Float32, '/proc_signal', 10)

        # Variables para almacenar los valores recibidos
        self.current_signal = 0.0
        self.current_time = 0.0
        
        # Parámetro de desplazamiento de fase (puedes cambiarlo)
        self.phase_shift = np.pi / 4  # 45 grados de fase
        
        # Temporizador para procesar la señal a 10 Hz
        self.timer = self.create_timer(0.1, self.process_signal)

    def signal_callback(self, msg):
        self.current_signal = msg.data  # Almacenar la señal recibida

    def time_callback(self, msg):
        self.current_time = msg.data  # Almacenar el tiempo recibido

    def process_signal(self):
        # Aplicar desplazamiento de fase
        shifted_signal = np.sin(self.current_time + self.phase_shift)

        # Reducir la amplitud a la mitad
        reduced_signal = shifted_signal / 2.0

        # Aplicar un offset para que siempre sea positivo
        processed_signal = reduced_signal + 0.5  # Asegura valores positivos

        # Crear y publicar el mensaje procesado
        proc_msg = Float32()
        proc_msg.data = float(processed_signal)
        self.proc_signal_publisher.publish(proc_msg)

        # Imprimir el resultado en la terminal
        self.get_logger().info(f'Processed Signal: {processed_signal:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SignalProcessor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped manually")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
