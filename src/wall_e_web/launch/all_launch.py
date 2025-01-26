from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': 9090
            }]
        ),
        Node(
            package='wall_e_web',
            executable='serve_web',
            name='web_server',
            output='screen'
        )
    ])
