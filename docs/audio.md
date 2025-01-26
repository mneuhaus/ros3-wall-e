# Audio System

 ## Overview
 The audio system provides Wall-E's voice and sound effects capabilities through a ROS2 node. It manages playback of pre-recorded sounds including startup sequences, voice clips, and status notifications.

 ## Hardware Setup
 - **Audio Output**: I2S audio via CM4's built-in audio interface
 - **Speaker**: 3W 4Ω speaker with amplifier
 - **Sound Files**: Pre-recorded MP3s stored in package resources

 ## Available Sounds
 | Filename        | Duration | Description                    |
 |----------------|----------|--------------------------------|
 | startup.mp3    | 2.1s     | Boot sequence sound            |
 | wall-e-1.mp3   | 1.5s     | Main voice clip               |
 | wall-e-4.mp3   | 1.2s     | Alternative voice clip        |
 | gorgeus.mp3    | 0.8s     | Admiration sound              |

 ## ROS2 Interface

 ### Node: audio_node
 - **Package**: audio
 - **Executable**: audio_node

 ### Parameters
 | Parameter      | Type    | Default | Description              |
 |---------------|---------|---------|--------------------------|
 | startup_sound | bool    | true    | Play sound on node start |
 | volume        | float   | 1.0     | Master volume (0.0-1.0)  |

 ### Topics
 | Topic         | Type              | Direction | Description        |
 |---------------|-------------------|-----------|-------------------|
 | /play_sound   | std_msgs/String   | Sub      | Play sound by name |
 | /stop_sound   | std_msgs/Empty    | Sub      | Stop playback     |

 ### Usage Examples
 ```bash
 # Play startup sound
 ros2 topic pub /play_sound std_msgs/String "data: 'startup.mp3'"

 # Stop current playback
 ros2 topic pub /stop_sound std_msgs/Empty
 ```

 ## Dependencies
 - ROS2 Humble
 - pygame (audio playback)
 - ALSA audio system

 ## Configuration
 The node requires proper ALSA configuration for the I2S audio interface. Ensure the following in `/etc/asound.conf`:

 ```
 pcm.!default {
     type hw
     card 0
 }

 ctl.!default {
     type hw
     card 0
 }
 ```