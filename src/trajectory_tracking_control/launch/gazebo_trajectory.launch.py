from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package paths
    gazebo_pkg = get_package_share_directory('gazebo_environment')
    trajectory_pkg = get_package_share_directory('trajectory_tracking_control')
    
    return LaunchDescription([
        # 1. Launch Gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'sonoma.launch.py')
            )
        ),

        # 2. Waypoint Reader (publishes target waypoints)
        Node(
            package='trajectory_tracking_control',
            executable='waypoint_reader',
            name='waypoint_reader',
            parameters=[{
                'waypoint_file': os.path.join(
                    trajectory_pkg,
                    'resource',
                    'sonoma_waypoints.txt'),
                'lookahead_distance': 3.0  # Constant value for basic Pure Pursuit
            }],
            output='screen'
        ),

        # 3. Pure Pursuit Controller (steering only)
        Node(
            package='trajectory_tracking_control',
            executable='pure_pursuit',
            name='pure_pursuit',
            parameters=[{
                'wheelbase': 2.5,          # Vehicle-specific parameter
                'constant_speed': 15     # Fixed speed (m/s)
            }],
            output='screen'
        ),

        Node(
            package='trajectory_tracking_control',
            executable='pid_controller',
            name='pid_controller',
            output='screen',
            parameters=[{
                'target_speed': 2.0,
                'Kp': 0.5,
                'Ki': 0.01,
                'Kd': 0.1
            }]
        ),
        
        # Command Mixer Node
        Node(
            package='trajectory_tracking_control',
            executable='cmd_mixer',
            name='cmd_mixer',
            output='screen'
        )
    ])