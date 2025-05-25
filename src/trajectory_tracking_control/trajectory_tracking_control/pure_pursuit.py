import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
import math

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit')
        self.waypoint_sub = self.create_subscription(PointStamped, '/waypoints', self.waypoint_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.constant_speed = 0
        self.lookahead_distance = 2
        self.current_pose = None
        
    def odom_callback(self, msg):
        self.current_speed = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.current_pose=msg.pose.pose
        
    def waypoint_callback(self, msg):
        if self.current_pose is None:
            return
            
        target_point = msg.point
        current_pos = self.current_pose.position
        current_orientation = self.current_pose.orientation
        
        # Calculate angle to target
        dx = target_point.x - current_pos.x
        dy = target_point.y - current_pos.y
        
        # Convert quaternion to yaw
        t3 = +2.0 * (current_orientation.w * current_orientation.z + 
             current_orientation.x * current_orientation.y)
        t4 = +1.0 - 2.0 * (current_orientation.y**2 + current_orientation.z**2)
        yaw = math.atan2(t3, t4)

        # Calculate alpha (angle between car heading and target direction)
        alpha = math.atan2(dy, dx) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        
        # Calculate steering angle using Pure Pursuit formula
        L = math.hypot(dx, dy)
        steering_angle = math.atan2(2.0 * 2.5 * math.sin(alpha), L)  # 2.5 is wheelbase
        
        # Publish command
        cmd = Twist()
        #cmd.linear.x = self.constant_speed #comment out when using PDI controller
        cmd.angular.z = steering_angle
        self.cmd_pub.publish(cmd)
    
def main(args=None):
    rclpy.init(args=args)
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()