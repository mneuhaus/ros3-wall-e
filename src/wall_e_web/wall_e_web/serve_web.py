#!/usr/bin/env python3
"""
Web server node for Wall-E web interface.

Serves static web content and handles HTTP requests.
"""
import http.server
import socketserver
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server')
        self.declare_parameter('port', 8000)
        self.port = self.get_parameter('port').value
        self.web_dir = os.path.join(get_package_share_directory('wall_e_web'), 'web')
        
        # Change to web directory
        os.chdir(self.web_dir)
        
        # Setup HTTP server
        handler = http.server.SimpleHTTPRequestHandler
        self.httpd = socketserver.TCPServer(("", self.port), handler)
        
        # Create timer to process HTTP requests
        self.create_timer(0.1, self.serve_requests)
        self.get_logger().info(f"Serving Wall-E web interface at http://localhost:{self.port}")

    def serve_requests(self):
        self.httpd.handle_request()

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
