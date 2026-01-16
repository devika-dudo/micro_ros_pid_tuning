#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32
import math

def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))

class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        
        # Declare parameters
        self.declare_parameter('signal_type', 'step')
        self.declare_parameter('amplitude', 90.0)
        self.declare_parameter('omega', 1.0)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('step_time', 1.0)
        self.declare_parameter('target_angle', 0.0)
        self.declare_parameter('min_angle', -360.0)
        self.declare_parameter('max_angle', 360.0)

        # Get initial values
        self.signal_type = self.get_parameter('signal_type').value
        self.amplitude = self.get_parameter('amplitude').value
        self.omega = self.get_parameter('omega').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.step_time = self.get_parameter('step_time').value
        self.target_angle = self.get_parameter('target_angle').value
        self.min_angle = self.get_parameter('min_angle').value
        self.max_angle = self.get_parameter('max_angle').value

        # Register callback
        self.add_on_set_parameters_callback(self.param_callback)

        # Publisher
        self.target_pub = self.create_publisher(Float32, '/motor/target_angle_cmd', 10)

        # Subscriber
        self.current_angle_sub = self.create_subscription(
            Float32, '/motor/current_angle', self.current_angle_callback, 10)
        
        self.current_angle = 0.0

        # Timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.current_time = 0.0

        self.get_logger().info('Signal Generator started!')
        self.log_current_params()

    def param_callback(self, params):
        for param in params:
            if param.name == 'signal_type':
                self.signal_type = param.value
                self.current_time = 0.0
            elif param.name == 'amplitude':
                self.amplitude = param.value
            elif param.name == 'omega':
                self.omega = param.value
            elif param.name == 'publish_rate':
                self.publish_rate = max(param.value, 1.0)
                new_period = 1.0 / self.publish_rate
                self.timer.cancel()
                self.timer = self.create_timer(new_period, self.timer_callback)
            elif param.name == 'step_time':
                self.step_time = param.value
            elif param.name == 'target_angle':
                self.target_angle = clamp(param.value, self.min_angle, self.max_angle)
            elif param.name == 'min_angle':
                self.min_angle = param.value
            elif param.name == 'max_angle':
                self.max_angle = param.value
        
        self.log_current_params()
        return SetParametersResult(successful=True)

    def timer_callback(self):
        self.current_time += 1.0 / self.publish_rate
        target = self.compute_signal(self.current_time)
        
        msg = Float32()
        msg.data = target
        self.target_pub.publish(msg)
        
        if int(self.current_time * 10) % 10 == 0:
            error = target - self.current_angle
            self.get_logger().info(
                f"t={self.current_time:.1f}s | target={target:.2f}° | "
                f"current={self.current_angle:.2f}° | error={error:.2f}°")

    def compute_signal(self, t):
        if self.signal_type == 'manual':
            val = self.target_angle
        elif self.signal_type == 'step':
            val = self.amplitude if t >= self.step_time else 0.0
        elif self.signal_type == 'square':
            if abs(self.omega) < 1e-9:
                val = self.amplitude
            else:
                period = (2.0 * math.pi) / self.omega
                time_in_period = t % period
                half_period = period / 2.0
                val = self.amplitude if time_in_period < half_period else -self.amplitude
        elif self.signal_type == 'sine':
            val = self.amplitude * math.sin(self.omega * t)
        else:
            self.get_logger().warn(f"Unknown signal_type '{self.signal_type}'")
            val = 0.0
        
        return clamp(val, self.min_angle, self.max_angle)

    def current_angle_callback(self, msg):
        self.current_angle = msg.data

    def log_current_params(self):
        self.get_logger().info(
            f"Params: type={self.signal_type}, amp={self.amplitude}°, "
            f"omega={self.omega}, rate={self.publish_rate}Hz")

def main(args=None):
    rclpy.init(args=args)
    node = SignalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Signal Generator.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
