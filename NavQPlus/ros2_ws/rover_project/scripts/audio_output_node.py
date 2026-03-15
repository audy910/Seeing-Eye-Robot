#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
import soundfile as sf
import numpy as np
import threading
import os
from ament_index_python.packages import get_package_share_directory

class AudioOutputNode(Node):

    def __init__(self):
        super().__init__('audio_output_node')

        # Set output device
        sd.default.device = None

        pkg_share = get_package_share_directory('rover_project')
        self.audio_dir = os.path.join(pkg_share, 'audio_clips')

        # Pre-load all clips into memory at startup
        self.clips = {}
        self.load_clips()

        self.subscription = self.create_subscription(
            String,
            'robot_commands',
            self.command_callback,
            10
        )

        self.subscription = self.create_subscription(
            String,
            'robot_voice_trigger',
            self.command_callback,
            10
        )

        self.get_logger().info("Audio output node ready.")

    def load_clips(self):
        """Load all wav files from audio_clips dir into memory at startup."""
        for fname in os.listdir(self.audio_dir):
            if fname.endswith('.wav'):
                cmd = fname.replace('.wav', '')
                path = os.path.join(self.audio_dir, fname)
                data, samplerate = sf.read(path, dtype='float32')
                self.clips[cmd] = (data, samplerate)
                self.get_logger().info(f"Loaded clip: {cmd}")

    def command_callback(self, msg):
        cmd = msg.data
        if cmd in self.clips:
            # Play in a separate thread so we don't block the ROS2 executor
            t = threading.Thread(target=self.play_clip, args=(cmd,), daemon=True)
            t.start()
        else:
            self.get_logger().warn(f"No audio clip found for command: '{cmd}'")

    def play_clip(self, cmd):
        data, samplerate = self.clips[cmd]
        self.get_logger().info(f"Playing clip for: '{cmd}'")
        sd.play(data, samplerate=samplerate)
        sd.wait()

    def destroy_node(self):
        sd.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioOutputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()