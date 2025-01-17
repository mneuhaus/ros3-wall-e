import http.server
import socketserver
import os
from ament_index_python.packages import get_package_share_directory

def main():
    PORT = 8000
    # Get the installed path for the package
    web_dir = os.path.join(get_package_share_directory('wall_e_web'), 'web')
    os.chdir(web_dir)

    handler = http.server.SimpleHTTPRequestHandler
    with socketserver.TCPServer(("", PORT), handler) as httpd:
        print(f"Serving Wall-E web interface at http://localhost:{PORT}")
        httpd.serve_forever()
