import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  

class MotorPublisher(Node):
    def __init__(self):
        super().__init__('motor_publisher')
        self.publisher_ = self.create_publisher(Float32, 'cmd_pwm', 10)
        self.get_logger().info('Tópico /cmd_pwm creado y listo para publicar.')

def main(args=None):
    rclpy.init(args=args)
    node = MotorPublisher()
    try:
        rclpy.spin(node)  # Mantiene el nodo activo sin publicar nada
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
