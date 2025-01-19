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
        ),
        
        # Start the teleop_twist_joy node with configuration
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[config_file],
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
            name='servo_2040_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyACM0'},
                {'baudrate': 115200}
            ]
        ),
    ])
