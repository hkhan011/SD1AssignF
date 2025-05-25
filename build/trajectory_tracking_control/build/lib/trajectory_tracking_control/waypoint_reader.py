import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped, Point
import numpy as np
from glob import glob
from ament_index_python.packages import get_package_share_directory
import os

class WaypointReader(Node):
	def __init__(self):
		super().__init__('waypoint_reader')
		package_dir = get_package_share_directory('trajectory_tracking_control')
		waypoint_file = os.path.join(package_dir, 'resource', 'sonoma_waypoints.txt')
		self.waypoints = self.load_waypoints(waypoint_file)
		# subscribing to odom to get current vehicle position
		self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
		# publisher target waypoints
		self.waypoint_pub = self.create_publisher(PointStamped, '/waypoints', 10)
		self.lookahead_distance = 20.0 # constant lookahead distance

	def load_waypoints(self, file):
		with open(file, 'r') as f:
			lines = f.readlines()
			# splits lines by commas and converts into floats
			return [list(map(float, line.strip().split(','))) for line in lines]\

	def odom_callback(self, msg):
		current_pos = msg.pose.pose.position
		# nearest waypoint to curr position
		nearest_wp = int(self.find_nearest_waypoint(current_pos)[0])
		# finds target point using the lookahead point
		target_wp = int(self.find_lookahead_point(nearest_wp))

		target_point = PointStamped()
		target_point.header.stamp = self.get_clock().now().to_msg()
		target_point.header.frame_id = "odom"
		# only use (x,y) as z cord is redundant in 2D control
		target_point.point = Point(x=float(self.waypoints[target_wp][0]), y=float(self.waypoints[target_wp][1]), z=0.0)
		self.waypoint_pub.publish(target_point)

	def find_nearest_waypoint(self, current_pos):
		distances = [np.hypot(wp[0]-current_pos.x, wp[1]-current_pos.y) for wp in self.waypoints]
		nearest_wp = np.argmin(distances)
		return nearest_wp, distances[nearest_wp]
	
	def find_lookahead_point(self, nearest_wp):
		lookahead_wp = nearest_wp
		while (lookahead_wp < len(self.waypoints) - 1 and 
           np.hypot(self.waypoints[lookahead_wp][0] - self.waypoints[nearest_wp][0],
                   self.waypoints[lookahead_wp][1] - self.waypoints[nearest_wp][1]) < self.lookahead_distance):
				   lookahead_wp += 1
		return lookahead_wp

def main(args=None):
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=args)
    waypoint_reader = WaypointReader()
    rclpy.spin(waypoint_reader)
    waypoint_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 