#!/usr/bin/env python3
"""
Audio playback node for Wall-E robot.

Handles playback of sound effects and background music.
"""

import os
import pygame
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


class AudioNode(Node):
    def __init__(self):
        super().__init__('audio_node')
        self.setup_hardware()
        self.get_logger().info('Audio node started')
        self.play_startup_sound()
        
    def setup_hardware(self):
        """Initialize audio hardware and drivers."""
        pygame.mixer.init()
        self.sounds_dir = os.path.join(
            get_package_share_directory('audio'),
            'sounds'
        )
        
    def play_startup_sound(self):
        """Play the startup sound."""
        startup_sound = os.path.join(self.sounds_dir, 'startup.mp3')
        if os.path.exists(startup_sound):
            sound = pygame.mixer.Sound(startup_sound)
            sound.play()
            self.get_logger().info('Playing startup sound')
        else:
            self.get_logger().error(f'Startup sound not found at {startup_sound}')
        
    def create_subscriptions(self):
        """Setup ROS2 subscriptions."""
        pass  # TODO: Add subscriptions for audio control
        
    def create_publishers(self):
        """Setup ROS2 publishers."""
        pass  # TODO: Add status publishers


def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
