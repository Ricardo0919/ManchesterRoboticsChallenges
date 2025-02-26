# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from rcl_interfaces.msg import SetParametersResult
from custom_interfaces.srv import SetProcessBool

# Class Definition
class DCMotor(Node):
    def __init__(self):
        super().__init__('dc_motor')

        # Declare parameters
        self.declare_parameter('sample_time', 0.01)  # System sample time in seconds
        self.declare_parameter('sys_gain_K', 2.16)   # System gain K
        self.declare_parameter('sys_tau_T', 0.05)    # System time constant Tau
        self.declare_parameter('initial_conditions', 0.0)  # System initial conditions

        # Get parameter values
        self.sample_time = self.get_parameter('sample_time').value
        self.param_K = self.get_parameter('sys_gain_K').value
        self.param_T = self.get_parameter('sys_tau_T').value
        self.initial_conditions = self.get_parameter('initial_conditions').value

        # Initialize messages and variables
        self.motor_output_msg = Float32()
        self.input_u = 0.0
        self.output_y = self.initial_conditions
        self.simulation_running = False
    
        # Declare publishers, subscribers, and timers
        self.motor_input_sub = self.create_subscription(Float32, 'motor_input_u', self.input_callback, 10)
        self.motor_speed_pub = self.create_publisher(Float32, 'motor_speed_y', 10)
        self.control_pub = self.create_publisher(Bool, 'simulation_control', 10)  # Simulation control publisher
        self.timer = self.create_timer(self.sample_time, self.timer_cb)

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Service server
        self.srv = self.create_service(SetProcessBool, 'StartMotor', self.simulation_service_callback)

        # Log node start
        self.get_logger().info('Dynamical System Node Started 🚀')   
    
    # Timer Callback
    def timer_cb(self):
        if not self.simulation_running:
            return  # Stop processing if simulation is not running    
        
        # DC Motor Simulation Equation:
        # y[k+1] = y[k] + ((-1/T) y[k] + (K/T) u[k]) * Ts
        self.output_y += (-1.0 / self.param_T * self.output_y + self.param_K / self.param_T * self.input_u) * self.sample_time 
        
        # Publish result
        self.motor_output_msg.data = self.output_y
        self.motor_speed_pub.publish(self.motor_output_msg)

    # Subscriber Callback
    def input_callback(self, input_sgn):
        self.input_u = input_sgn.data

    # Service Callback to Start/Stop Simulation
    def simulation_service_callback(self, request, response):
        self.simulation_running = request.enable
        action = "Started" if self.simulation_running else "Stopped"
        self.get_logger().info(f"{('🚀' if self.simulation_running else '🛑')} Simulation {action}")
        
        response.success = True
        response.message = f"Simulation {action} Successfully"
        
        # Publish simulation state
        control_msg = Bool()
        control_msg.data = self.simulation_running
        self.control_pub.publish(control_msg)
        
        return response

    # Parameter Callback
    def parameters_callback(self, params):
        for param in params:
            if param.name == "sys_gain_K":
                if param.value < 0.0:
                    self.get_logger().warn("Invalid sys_gain_K! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="sys_gain_K cannot be negative")
                self.param_K = param.value  # Update internal variable
                self.get_logger().info(f"sys_gain_K updated to {self.param_K}")
                
            if param.name == "sys_tau_T":
                if param.value < 0.0:
                    self.get_logger().warn("Invalid sys_tau_T! It cannot be negative.")
                    return SetParametersResult(successful=False, reason="sys_tau_T cannot be negative")
                self.param_T = param.value  # Update internal variable
                self.get_logger().info(f"sys_tau_T updated to {self.param_T}")

        return SetParametersResult(successful=True)

# Main
def main(args=None):
    rclpy.init(args=args)
    node = DCMotor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

# Execute Node
if __name__ == '__main__':
    main()