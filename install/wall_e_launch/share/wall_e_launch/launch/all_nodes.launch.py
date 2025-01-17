# wall_e_control.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
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
    ])
