import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # PID Parameters
        self.target_speed = 2.0  # m/s (setpoint)
        self.Kp = 0.5
        self.Ki = 0.01
        self.Kd = 0.1
        
        # State
        self.integral = 0.0
        self.prev_error = 0.0
        
        # Subscriber (vehicle speed from odometry)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Publisher for SPEED ONLY (new topic)
        self.speed_pub = self.create_publisher(Twist, '/speed_cmd', 10)

    def odom_callback(self, msg):
        current_speed = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y)
        
        # PID Calculations
        error = self.target_speed - current_speed
        self.integral += error
        derivative = error - self.prev_error
        
        # Anti-windup (limit integral term)
        self.integral = max(min(self.integral, 2.0), -2.0)
        
        # Compute acceleration command
        acceleration = (self.Kp * error + 
                       self.Ki * self.integral + 
                       self.Kd * derivative)
        
        # Publish SPEED ONLY (no steering)
        speed_msg = Twist()
        speed_msg.linear.x = max(0, current_speed + acceleration)  # Prevent reverse
        self.speed_pub.publish(speed_msg)
        
        self.prev_error = error

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    try:
        rclpy.spin(pid_controller)
    except KeyboardInterrupt:
        pass
    finally:
        pid_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()