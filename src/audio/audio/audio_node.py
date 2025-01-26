#!/usr/bin/env python3
"""
Audio playback node for Wall-E robot.

Handles playback of sound effects and background music.
"""

import os
import time
import random
import pygame
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory


class AudioNode(Node):
    def __init__(self):
        super().__init__('audio_node')
        self.setup_hardware()
        self.create_subscriptions()
        self.get_logger().info('Audio node started')
        self.play_startup_sound()
        
    def setup_hardware(self):
        """Initialize audio hardware and drivers."""
        pygame.mixer.init()
        # Set both mixer and music volume to maximum
        pygame.mixer.music.set_volume(1.0)
        for i in range(pygame.mixer.get_num_channels()):
            pygame.mixer.Channel(i).set_volume(1.0)
        self.sounds_dir = os.path.join(
            get_package_share_directory('audio'),
            'sounds'
        )
        
    def play_startup_sound(self) -> None:
        """Play the startup sound followed by a random sound."""
        # Play startup sound
        startup_sound = os.path.join(self.sounds_dir, 'startup.mp3')
        if not os.path.exists(startup_sound):
            self.get_logger().error(f'Startup sound not found at {startup_sound}')
            return
            
        try:
            sound = pygame.mixer.Sound(startup_sound)
            sound.set_volume(1.0)  # Set individual sound volume to maximum
            channel = pygame.mixer.find_channel()
            if not channel:
                self.get_logger().error('No available audio channels')
                return
                
            channel.set_volume(1.0)
            channel.play(sound)
            self.get_logger().info('Playing startup sound')
            # Wait for startup sound to finish
            time.sleep(2)
        except pygame.error as e:
            self.get_logger().error(f'Error playing startup sound: {e}')
        
    def play_sound_callback(self, msg):
        """Handle sound playback requests."""
        sound_file = os.path.join(self.sounds_dir, msg.data)
        if not os.path.exists(sound_file):
            self.get_logger().error(f'Sound file not found: {sound_file}')
            return
            
        try:
            sound = pygame.mixer.Sound(sound_file)
            sound.set_volume(1.0)
            channel = pygame.mixer.find_channel()
            if channel:
                channel.set_volume(1.0)
                channel.play(sound)
                self.get_logger().info(f'Playing sound: {msg.data}')
        except pygame.error as e:
            self.get_logger().error(f'Error playing sound {msg.data}: {e}')
    
    def play_random_sound(self):
        """Play a random sound from the available MP3 files."""
        sound_files = [f for f in os.listdir(self.sounds_dir) if f.endswith('.mp3') and f != 'startup.mp3']
        if sound_files:
            random_sound = os.path.join(self.sounds_dir, random.choice(sound_files))
            sound = pygame.mixer.Sound(random_sound)
            sound.set_volume(1.0)  # Set individual sound volume to maximum
            channel = pygame.mixer.find_channel()
            if channel:
                channel.set_volume(1.0)
                channel.play(sound)
            self.get_logger().info(f'Playing random sound: {os.path.basename(random_sound)}')

    def create_subscriptions(self):
        """Setup ROS2 subscriptions."""
        self.play_sound_sub = self.create_subscription(
            String,
            '/play_sound',
            self.play_sound_callback,
            10
        )
        
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
