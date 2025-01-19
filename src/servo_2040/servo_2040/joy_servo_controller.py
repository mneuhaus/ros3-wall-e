import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

class JoyServoController(Node):
    def __init__(self):
        super().__init__('joy_servo_controller')
        
        # Default servo positions (middle positions)
        self.servo_positions = [45.0, 45.0, 20.0, 20.0, 45.0, 90.0, 90.0, 90.0, 90.0]
        
        # Create publisher for servo commands
        self.servo_publisher = self.create_publisher(
            Float32MultiArray,
            'servo_commands',
            10
        )
        
        # Subscribe to joy messages
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Button mappings
        self.button_map = {
            0: {'servo': 0, 'direction': 1},   # A button - eyebrow left up
            1: {'servo': 0, 'direction': -1},  # B button - eyebrow left down
        }
        
        # Movement increment in degrees
        self.movement_increment = 5.0
        
        # Servo limits
        self.servo_limits = [
            (0, 90),   # eyebrow_left
            (0, 90),   # eyebrow_right
            (0, 40),   # head_left
            (0, 40),   # head_right
            (0, 90),   # neck_tilt
            (0, 180),  # neck_raise
            (0, 180),  # neck_pan
            (0, 180),  # arm_left
            (0, 180),  # arm_right
        ]

    def joy_callback(self, msg):
        position_changed = False
        
        # Check each mapped button
        for button_idx, mapping in self.button_map.items():
            if button_idx < len(msg.buttons) and msg.buttons[button_idx]:
                servo_idx = mapping['servo']
                direction = mapping['direction']
                
                # Calculate new position
                new_position = self.servo_positions[servo_idx] + (direction * self.movement_increment)
                
                # Apply limits
                min_val, max_val = self.servo_limits[servo_idx]
                new_position = max(min_val, min(max_val, new_position))
                
                # Update position if changed
                if new_position != self.servo_positions[servo_idx]:
                    self.servo_positions[servo_idx] = new_position
                    position_changed = True
        
        # Publish new positions if any changed
        if position_changed:
            command_msg = Float32MultiArray()
            command_msg.data = self.servo_positions
            self.servo_publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
