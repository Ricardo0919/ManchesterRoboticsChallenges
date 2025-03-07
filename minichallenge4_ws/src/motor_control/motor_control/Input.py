import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult

class SetPointPublisher(Node):
    def __init__(self):
        super().__init__('set_point_node')

        self.declare_parameter('speed', 10.0)
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('signal_type', 'sine')
        
        self.speed = self.get_parameter('speed').value
        self.frequency = self.get_parameter('frequency').value
        self.signal_type = self.get_parameter('signal_type').value
        
        self.validate_parameters()
        
        self.timer_period = 0.1
        self.signal_publisher = self.create_publisher(Float32, 'set_point', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_cb)
        
        self.signal_msg = Float32()
        self.start_time = self.get_clock().now()

        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info("SetPoint Node Started 🚀")

    def validate_parameters(self):
        """Valida y ajusta los parámetros speed y signal_type si están fuera de los valores permitidos."""
        if not (-15 <= self.speed <= 15):
            self.get_logger().warn(f"Valor de speed fuera de rango ({self.speed}), manteniendo último válido.")
            self.speed = self.get_parameter('speed').value  # Mantiene el último válido
        
        if self.signal_type not in ['sine', 'square', 'step']:
            self.get_logger().warn(f"Tipo de señal inválido ({self.signal_type}), manteniendo último válido.")
            self.signal_type = self.get_parameter('signal_type').value  # Mantiene el último válido

    def timer_cb(self):
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if self.signal_type == 'sine':
            self.signal_msg.data = self.speed * np.sin(self.frequency * elapsed_time)
        elif self.signal_type == 'square':
            self.signal_msg.data = self.speed if np.sin(self.frequency * elapsed_time) >= 0 else -self.speed
        elif self.signal_type == 'step':
            self.signal_msg.data = self.speed
        else:
            self.get_logger().warn(f"Tipo de señal desconocido: {self.signal_type}")
            return

        self.signal_publisher.publish(self.signal_msg)

    def parameters_callback(self, params):
        for param in params:
            if param.name == "speed":
                if -18 <= param.value <= 18:
                    self.speed = param.value
                    self.get_logger().info(f"Velocidad actualizada a {self.speed} rad/s")
                else:
                    self.get_logger().warn(f"Valor de speed fuera de rango ({param.value} rad/s), manteniendo {self.speed} rad/s")

            if param.name == "frequency":
                self.frequency = param.value
                self.get_logger().info(f"Frecuencia actualizada a {self.frequency}")

            if param.name == "signal_type":
                if param.value in ['sine', 'square', 'step']:
                    self.signal_type = param.value
                    self.get_logger().info(f"Tipo de señal actualizado a {self.signal_type}")
                else:
                    self.get_logger().warn(f"Tipo de señal inválido ({param.value}), manteniendo {self.signal_type}")

        return SetParametersResult(successful=True)

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
