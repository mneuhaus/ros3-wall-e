from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

config_file = os.path.join(
    os.path.expanduser('~'),
    'ros2_ws',
    'src',
    'teleop_twist_joy',
    'config',
    'teleop_twist_joy.yaml'
)

def generate_launch_description():
    return LaunchDescription([
        # Start the joystick node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'device_id': 0}],  # Explicitly set device ID
            arguments=['--ros-args', '--log-level', 'info'],
        ),
        
        # Start the teleop_twist_joy node with configuration
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[config_file],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[
                ('/cmd_vel', '/cmd_vel')  # Ensures that the command is published on /cmd_vel
            ]
        ),
        
        # Start the motor controller node
        # Node(
        #     package='motor_controller',
        #     executable='motor_controller_node',
        #     name='motor_controller_node',
        #     output='screen',
        #     parameters=[
        #         {'port': '/dev/ttyAMA2'},  # Adjust this if using a different serial port
        #     ]
        # ),
        # Start the servo_2040 node
        Node(
            package='servo_2040',
            executable='servo_2040',
            name='servo_2040',
            output='screen',
            respawn=True,
            respawn_delay=1,
            parameters=[
                {'serial_port': '/dev/ttyACM0'},  # Use simpler device name
                {'baudrate': 115200}
            ],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        
        # Start the web server and rosbridge
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': 9091  # Changed port to avoid conflicts
            }]
        ),
        Node(
            package='wall_e_web',
            executable='serve_web',
            name='web_server',
            output='screen'
        ),
        
        # Start the audio node
        Node(
            package='audio',
            executable='audio_node',
            name='audio_node',
            output='screen',
            parameters=[{
                'startup_sound': True,
                'volume': 1.0
            }]
        ),
        
        # Start the power monitoring node
        Node(
            package='power',
            executable='power_monitor',
            name='power_monitor',
            output='screen',
            respawn=True,  # Automatically restart if it crashes
            respawn_delay=1,  # Wait 1 second before restarting
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])
