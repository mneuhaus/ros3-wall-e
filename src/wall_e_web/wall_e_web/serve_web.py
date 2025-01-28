#!/usr/bin/env python3
"""
Web server node for Wall-E web interface.

Serves static web content and handles HTTP requests.
"""
import http.server
import socketserver
import socket
import os
import rclpy
import subprocess
from rclpy.node import Node
from std_msgs.msg import Int32
from ament_index_python.packages import get_package_share_directory

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server')
        self.declare_parameter('port', 8000)
        self.port = self.get_parameter('port').value
        self.web_dir = os.path.join(get_package_share_directory('wall_e_web'), 'web')
        
        # Change to web directory
        os.chdir(self.web_dir)
        
        # Setup HTTP server with custom handler that doesn't block
        class NonBlockingHTTPServer(socketserver.TCPServer):
            def server_bind(self):
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket.settimeout(0)  # Non-blocking
                socketserver.TCPServer.server_bind(self)

        handler = http.server.SimpleHTTPRequestHandler
        self.httpd = NonBlockingHTTPServer(("", self.port), handler)
        
        # Create timer to process HTTP requests
        # Create subscription for volume control
        self.create_subscription(
            Int32,
            'set_volume',
            self.volume_callback,
            10
        )
        
        self.create_timer(0.1, self.serve_requests)
        self.get_logger().info(f"Serving Wall-E web interface at http://localhost:{self.port}")
        
    def volume_callback(self, msg):
        """Handle volume control messages."""
        try:
            # Set ALSA Master volume
            volume = max(0, min(100, msg.data))  # Clamp between 0-100
            subprocess.run(['amixer', 'set', 'Master', f'{volume}%'], 
                         capture_output=True, text=True, check=True)
            self.get_logger().info(f'Volume set to {volume}%')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Failed to set volume: {e}')

    def serve_requests(self):
        try:
            self.httpd.handle_request()
        except socket.timeout:
            pass  # Expected timeout for non-blocking socket
        except Exception as e:
            self.get_logger().error(f"HTTP server error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
