#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32

class PIDParamPublisher(Node):
    def __init__(self):
        super().__init__('pid_param_publisher')
        
        # Declare PID parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('publish_rate', 1.0)
        
        # Get initial values
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # Register callback
        self.add_on_set_parameters_callback(self.param_callback)
        
        # Create publishers
        self.kp_pub = self.create_publisher(Float32, '/motor/pid/kp', 10)
        self.ki_pub = self.create_publisher(Float32, '/motor/pid/ki', 10)
        self.kd_pub = self.create_publisher(Float32, '/motor/pid/kd', 10)
        
        # Timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_params)
        
        # Publish initial values
        self.publish_params()
        
        self.get_logger().info('PID Parameter Publisher started!')
        self.get_logger().info(f'Initial: KP={self.kp}, KI={self.ki}, KD={self.kd}')
    
    def param_callback(self, params):
        for param in params:
            if param.name == 'kp':
                self.kp = param.value
                self.get_logger().info(f'KP changed to: {self.kp}')
            elif param.name == 'ki':
                self.ki = param.value
                self.get_logger().info(f'KI changed to: {self.ki}')
            elif param.name == 'kd':
                self.kd = param.value
                self.get_logger().info(f'KD changed to: {self.kd}')
            elif param.name == 'publish_rate':
                self.publish_rate = max(param.value, 0.1)
                new_period = 1.0 / self.publish_rate
                self.timer.cancel()
                self.timer = self.create_timer(new_period, self.publish_params)
        
        self.publish_params()
        return SetParametersResult(successful=True)
    
    def publish_params(self):
        kp_msg = Float32()
        kp_msg.data = self.kp
        self.kp_pub.publish(kp_msg)
        
        ki_msg = Float32()
        ki_msg.data = self.ki
        self.ki_pub.publish(ki_msg)
        
        kd_msg = Float32()
        kd_msg.data = self.kd
        self.kd_pub.publish(kd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDParamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down PID Parameter Publisher.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
