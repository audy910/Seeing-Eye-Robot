#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import soundfile as sf
import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import json
import numpy as np
from scipy.signal import resample_poly
import sys
import os
from ament_index_python.packages import get_package_share_directory
import time


DEVICE_ID = 1
RATE = 16000
CHANNELS = 1


class AudioInputNode(Node):

    def __init__(self):
        super().__init__('audio_input_node')
        sd.default.device = 'pulse'
        self.command_map = {
            "forward": "MOVE_FORWARD",
            "back": "MOVE_BACKWARD",
            "stop": "EMERGENCY_STOP",
            "left": "TURN_LEFT",
            "right": "TURN_RIGHT",
            "orbach": "ORBACH",
            "hub": "HUB",
            "borns": "BOURNS",
            "winston": "WCH",
            "rivera": "RIVERA",
            "bell": "BELLTOWER",
            "arrived" : "ARRIVED"
        }
        

        vocab_list = list(self.command_map.keys()) + ["[unk]"]
        vocab_json = json.dumps(vocab_list)
        
        self.get_logger().info(f"Listening ONLY for: {vocab_json}")

        self.text_pub = self.create_publisher(String, 'speech_text', 10)
        self.cmd_pub = self.create_publisher(String, 'robot_commands', 10)

        self.audio_queue = queue.Queue(maxsize=10)
        self.last_command = None
        self.last_command_time = 0
        self.command_cooldown = 4.0
        try:
            pkg_share = get_package_share_directory('rover_project')
            model = Model(os.path.join(pkg_share, 'models', 'vosk-model'))
            self.rec = KaldiRecognizer(model, RATE, vocab_json)
        except Exception as e:
            self.get_logger().error(f"Model load failed: {e}")
            raise

        self.stream = sd.InputStream(
            samplerate=48000,
            blocksize=8000,
            device=DEVICE_ID,
            dtype='int16',
            channels=CHANNELS,
            callback=self.callback
        )
        self.stream.start()

        self.timer = self.create_timer(0.01, self.process_audio)


    def callback(self, indata, frames, time_info, status):
        if status:
            print(status, file=sys.stderr)

        audio = np.frombuffer(indata, dtype=np.int16)
        audio = audio.reshape(-1, CHANNELS)
        mono = audio.mean(axis=1)
        downsampled = resample_poly(mono, up=1, down=3)

        try:
            self.audio_queue.put_nowait(downsampled.astype(np.int16).tobytes())
        except queue.Full:
            pass

    def process_audio(self):
        max_chunks = 5  # don't process more than 5 chunks per timer call
        chunks_processed = 0
        while not self.audio_queue.empty() and chunks_processed < max_chunks:
            data = self.audio_queue.get()
            chunks_processed += 1

            if self.rec.AcceptWaveform(data):
                result = json.loads(self.rec.Result())
                text = result.get("text", "").lower()

                if text:
                    self.get_logger().info(f"Heard: '{text}'")
                    
                    # Check if any part of the heard text is in our command map
                    words = text.split()
                    for word in words:
                        if word in self.command_map:
                            cmd_string = self.command_map[word]
                            self.execute_command(cmd_string)

    def execute_command(self, cmd_string):
        now = time.time()
        # Block same command within cooldown window
        if cmd_string == self.last_command and (now - self.last_command_time) < self.command_cooldown:
            self.get_logger().info(f"Ignoring duplicate command: '{cmd_string}'")
            return
    
        self.last_command = cmd_string
        self.last_command_time = now

        msg = String()
        msg.data = cmd_string
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Published command: '{cmd_string}'")

def main(args=None):
    rclpy.init(args=args)
    node = AudioInputNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()