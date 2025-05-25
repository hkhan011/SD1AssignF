import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdMixer(Node):
    def __init__(self):
        super().__init__('cmd_mixer')
        
        # Subscribers
        self.steer_sub = self.create_subscription(
            Twist, '/steer_cmd', self.steer_callback, 10)
        self.speed_sub = self.create_subscription(
            Twist, '/speed_cmd', self.speed_callback, 10)
        
        # Final command publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State
        self.current_steer = 0.0
        self.current_speed = 0.0

    def steer_callback(self, msg):
        self.current_steer = msg.angular.z
        self._publish_combined()

    def speed_callback(self, msg):
        self.current_speed = msg.linear.x
        self._publish_combined()

    def _publish_combined(self):
        cmd = Twist()
        cmd.linear.x = self.current_speed
        cmd.angular.z = self.current_steer
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    cmd_mixer = CmdMixer()
    try:
        rclpy.spin(cmd_mixer)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_mixer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()