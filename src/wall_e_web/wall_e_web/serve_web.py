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
        self.current_volume = self.get_current_volume()
        
        # Change to web directory
        os.chdir(self.web_dir)
        
        # Setup HTTP server with custom handler that doesn't block
        class NonBlockingHTTPServer(socketserver.TCPServer):
            def __init__(self, server_address, RequestHandlerClass, node):
                self.node = node
                super().__init__(server_address, RequestHandlerClass)

            def server_bind(self):
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket.settimeout(0)  # Non-blocking
                try:
                    self.socket.bind(('0.0.0.0', self.server_address[1]))
                    self.server_address = self.socket.getsockname()
                    self.node.get_logger().info(f"Server bound to {self.server_address}")
                except Exception as e:
                    self.node.get_logger().error(f"Failed to bind server: {e}")
                    raise

        class DebugHandler(http.server.SimpleHTTPRequestHandler):
            def log_message(self, format, *args):
                self.server.node.get_logger().info(f"HTTP {format%args}")
            
            def log_error(self, format, *args):
                self.server.node.get_logger().error(f"HTTP Error: {format%args}")

        try:
            self.httpd = NonBlockingHTTPServer(('0.0.0.0', self.port), DebugHandler, self)
            self.get_logger().info(f"Web server initialized on port {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to create HTTP server: {e}")
            raise
        
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
        
    def get_current_volume(self):
        """Get current system volume level."""
        try:
            result = subprocess.run(['pactl', 'get-sink-volume', '@DEFAULT_SINK@'], 
                                  capture_output=True, text=True, check=True)
            # Parse the volume percentage from output
            volume_str = result.stdout
            if 'Volume:' in volume_str:
                # Extract percentage from format like "Volume: front-left: 65536 / 100% / -0.00 dB"
                volume = int(volume_str.split('%')[0].split('/')[-1].strip())
                return volume
            return 80  # Default if parsing fails
        except subprocess.CalledProcessError:
            self.get_logger().warning("Failed to get current volume, using default")
            return 80

    def volume_callback(self, msg):
        """Handle volume control messages."""
        try:
            # Set PulseAudio volume using pactl
            volume = max(0, min(100, msg.data))  # Clamp between 0-100
            volume_float = volume / 100.0  # Convert to 0.0-1.0 range
            subprocess.run(['pactl', 'set-sink-volume', '@DEFAULT_SINK@', f'{volume_float:0.2f}'], 
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
            import traceback
            self.get_logger().error(traceback.format_exc())

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
