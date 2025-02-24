import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
import time

class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.signal_publisher = self.create_publisher(Float32, '/signal', 10)
        self.time_publisher = self.create_publisher(Float32, '/time', 10)
        self.timer = self.create_timer(0.1, self.publish_signal)  # 10 Hz
        self.start_time = time.time()

    def publish_signal(self):
        current_time = time.time() - self.start_time
        signal_value = np.sin(current_time)  # Usando numpy para generar la señal

        # Publicar tiempo
        time_msg = Float32()
        time_msg.data = float(current_time)
        self.time_publisher.publish(time_msg)

        # Publicar señal
        signal_msg = Float32()
        signal_msg.data = float(signal_value)
        self.signal_publisher.publish(signal_msg)

        # Imprimir en la terminal
        self.get_logger().info(f'Time: {current_time:.2f} s, Signal: {signal_value:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SignalGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped manually")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
