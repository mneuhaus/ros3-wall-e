# Wall-E Web Interface Documentation

## Overview
The wall_e_web package provides a web-based interface for controlling and monitoring Wall-E. It uses rosbridge for WebSocket communication between the browser and ROS2 system.

## Hardware Requirements
- Raspberry Pi CM4 running ROS2
- Network connection (WiFi or Ethernet)
- Web browser on client device

## Software Dependencies
- ROS2 Humble
- rosbridge_suite
- Python 3.8+
- Modern web browser with WebSocket support

## Configuration Parameters
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| port | int | 8000 | HTTP server port |
| rosbridge_port | int | 9090 | WebSocket port |
| web_root | string | web/ | Web content directory |

## Usage Instructions

### Starting the Server
```bash
ros2 launch wall_e_web all_launch.py
```

### Accessing the Interface
1. Connect to the same network as Wall-E
2. Open browser to: `http://<wall-e-ip>:8000`

### Web Interface Features
- Gamepad-style control layout
- Virtual joystick for movement control (left side)
- Action buttons for sounds and movements (right side)
- Head position controls
- Sound effect triggers
- Emergency stop button
- Connection status indicator

## Implementation Details

### Architecture
```
Browser <-> WebSocket <-> rosbridge <-> ROS2
```

### Security Considerations
- Local network access only
- No authentication (yet)
- Read-only system access

### ROS2 Topics
| Topic | Type | Access | Description |
|-------|------|--------|-------------|
| /cmd_vel | geometry_msgs/Twist | Pub | Motion control |
| /play_sound | std_msgs/String | Pub | Sound triggers |
| /battery_state | sensor_msgs/BatteryState | Sub | Power status |
| /diagnostics | diagnostic_msgs/DiagnosticArray | Sub | System health |

## Troubleshooting Guide

### Common Issues
1. **Cannot Connect**
   - Verify network connection
   - Check firewall settings
   - Confirm ports are open

2. **Video Feed Issues**
   - Check camera node status
   - Verify WebRTC configuration
   - Check network bandwidth

3. **Control Lag**
   - Monitor network latency
   - Check system resource usage
   - Verify WebSocket connection

### Browser Support
- Chrome/Chromium (recommended)
- Firefox
- Safari
- Edge

## API Documentation

### WebServerNode Class
Main ROS2 node serving web content.

#### Methods
- `__init__()`: Initialize web server
- `start_server()`: Begin HTTP service
- `handle_request()`: Process HTTP requests

### JavaScript API
The web interface provides a JavaScript API for ROS2 interaction:

```javascript
// Connect to ROS
ros.connect('ws://wall-e.local:9090')

// Movement control
publishVelocity(linear, angular)  // For joystick control
setServoPosition(servoIndex, position)  // For head movement

// Sound control
playSound(soundFile)  // Trigger sound effects

// Emergency stop
stopButton.onclick = function() {
    publishVelocity(0, 0)
    publishTrackSpeeds(0, 0)
}
```

## Development Guide

### Adding New Features
1. Add HTML/JS to web directory
2. Update rosbridge topics if needed
3. Test cross-browser compatibility
4. Update documentation

### Testing
```bash
# Run tests
ros2 test wall_e_web

# Test web server
ros2 run wall_e_web serve_web --ros-args -p port:=8080
```

### Building
```bash
colcon build --packages-select wall_e_web
```

## Performance Optimization
- Use WebSocket compression
- Minimize DOM updates
- Cache static resources
- Optimize image streaming

## Future Improvements
- User authentication
- HTTPS support
- Custom behavior programming
- Diagnostic data logging
