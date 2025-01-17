from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='servo_2040',
            executable='serial_listener',
            name='serial_listener',
            parameters=[
                {'serial_port': '/dev/ttyACM0', 'baudrate': 115200}
            ],
            output='screen',
        )
    ])
