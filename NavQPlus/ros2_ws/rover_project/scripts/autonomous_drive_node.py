#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Bool
from rover_project.msg import Proximity

# Command Table
CMD_STOP              = 2
CMD_FORWARD_STRAIGHT  = 3
CMD_FORWARD_RIGHT     = 4
CMD_FORWARD_LEFT      = 5
CMD_BACKWARD_STRAIGHT = 6
CMD_BACKWARD_RIGHT    = 7
CMD_BACKWARD_LEFT     = 8
CMD_RIGHT             = 9 
CMD_LEFT              = 10

# States
STATE_FORWARD    = "FORWARD"
STATE_CLIFF_STOP = "CLIFF_STOP"
STATE_REV_CHECK  = "REV_CHECK"
STATE_REVERSING  = "REVERSING"
STATE_LOOK_L     = "LOOK_L"
STATE_SCAN_L     = "SCAN_L"
STATE_LOOK_R     = "LOOK_R"
STATE_SCAN_R     = "SCAN_R"
STATE_DRIVE_OUT  = "DRIVE_OUT"


class AutonomousDriveNode(Node):
    def __init__(self):
        super().__init__('autonomous_drive_node')

        self.sub = self.create_subscription(Proximity, 'can/proximity_sensors', self.proximity_callback, 10)
        self.cmd_pub = self.create_publisher(Int32, 'nav/drive_cmd', 10)
        self.override_pub = self.create_publisher(Bool, 'safety/override_active', 1)
        self.voice_pub = self.create_publisher(String, 'robot_voice_trigger', 10)

        self.declare_parameter('invert_drive', False)
        self.declare_parameter('wall_threshold_mm', 400.0)
        self.declare_parameter('reverse_time_s', 1.0)
        self.declare_parameter('sensor_time_s', 0.5)
        self.declare_parameter('maneuver_time_s', 2.0)
        self.declare_parameter('required_readings', 2)
        self.declare_parameter('cliff_hold_s', 1.0)

        self.invert_drive = self.get_parameter('invert_drive').value
        self.wall_threshold = self.get_parameter('wall_threshold_mm').value
        self.reverse_time = self.get_parameter('reverse_time_s').value
        self.sensor_time = self.get_parameter('sensor_time_s').value
        self.maneuver_time = self.get_parameter('maneuver_time_s').value
        self.required_readings = self.get_parameter('required_readings').value
        self.cliff_hold_seconds = self.get_parameter('cliff_hold_s').value

        # Wall validation
        self.valid_reading_count = 2

        # Cliff latch
        self.cliff_active    = False
        self.last_cliff_time = 0.0
        self.current_state = STATE_FORWARD
        self.state_start_time = 0.0
        self.chosen_turn_cmd = CMD_FORWARD_LEFT
        self._last_override_state = None

    _INVERT_MAP = {
        CMD_FORWARD_STRAIGHT: CMD_BACKWARD_STRAIGHT,
        CMD_FORWARD_RIGHT: CMD_BACKWARD_RIGHT,
        CMD_FORWARD_LEFT: CMD_BACKWARD_LEFT,
        CMD_BACKWARD_STRAIGHT: CMD_FORWARD_STRAIGHT,
        CMD_BACKWARD_RIGHT: CMD_FORWARD_RIGHT,
        CMD_BACKWARD_LEFT: CMD_FORWARD_LEFT,
    }

    def trigger_voice(self, phrase: str):
        msg = String()
        msg.data = phrase
        self.voice_pub.publish(msg)
        self.get_logger().info(f"[VOICE] Triggered: {phrase}")

    def send_cmd(self, cmd: int):
        physical = self._INVERT_MAP.get(cmd, cmd) if self.invert_drive else cmd
        
        self.get_logger().info(f"[AUTONOMOUS DRIVING] Publishing CMD: {physical}")
        
        msg = Int32()
        msg.data = physical
        self.cmd_pub.publish(msg)

    def publish_override(self, active: bool):
        if self._last_override_state != active:
            if active:
                self.get_logger().info("[AUTONOMOUS] 🚨 TAKING CONTROL (Override ON)")
            else:
                self.get_logger().info("[AUTONOMOUS] 🤝 RELEASING CONTROL (Override OFF)")
            self._last_override_state = active

        msg = Bool()
        msg.data = active
        self.override_pub.publish(msg)

    def set_state(self, new_state):
        self.get_logger().info(f"Transitioning to: {new_state}")
        self.valid_reading_count = 0
        self.current_state = new_state
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9

    def proximity_callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.state_start_time

        # ── Cliff Detection ──
        if msg.cliff_detected:
            if not self.cliff_active:
                self.get_logger().warn("CLIFF! Emergency Stop")
                self.trigger_voice("Stop")
            self.cliff_active = True
            self.last_cliff_time = now
        elif self.cliff_active and (now - self.last_cliff_time) > self.cliff_hold_seconds:
            self.cliff_active = False

        if self.cliff_active:
            self.publish_override(True)
            self.send_cmd(CMD_STOP)
            if self.current_state != STATE_CLIFF_STOP:
                self.set_state(STATE_CLIFF_STOP)
                self.trigger_voice("Stop")
            return

        # ── Front sensor invalid ─────────────────────────────────────────────
        # HC-SR04 returns invalid when nothing is in range (open space).
        # In STATE_FORWARD: release override and return — waypoint_follower drives.
        # In correction states: fall through so time-based transitions (REVERSING,
        # DRIVE_OUT, etc.) still complete even when the front sensor reads nothing.
        if not msg.front_valid and self.current_state in (STATE_FORWARD, STATE_CLIFF_STOP):
            self.current_state = STATE_FORWARD  # cliff cleared with open space ahead
            self.publish_override(False)
            return

        # ── Wall validation (STATE_FORWARD only) ─────────────────────────────
        # Count consecutive valid readings below threshold (regardless of direction).
        # This correctly handles the case where waypoint_follower has already stopped
        # the rover — the distance stabilises rather than continuing to decrease.
        if self.current_state == STATE_FORWARD and msg.front_valid:
            if msg.proximity_front < self.wall_threshold:
                self.valid_reading_count += 1
            else:
                self.valid_reading_count = 0  # clear reading — reset hysteresis

        # ── State Machine ──
        if self.current_state == STATE_FORWARD:
            if self.valid_reading_count >= self.required_readings:
                self.get_logger().warn(f"Wall confirmed: {msg.proximity_front}mm")
                self.set_state(STATE_REV_CHECK)
            else:
                self.publish_override(False)
                return

        if self.current_state == STATE_CLIFF_STOP:
            self.set_state(STATE_REV_CHECK)

        self.publish_override(True)

        if self.current_state == STATE_REV_CHECK:
            rear_blocked = msg.rear_valid and (msg.proximity_rear <= self.wall_threshold)
            if not rear_blocked:
                self.set_state(STATE_REVERSING)
            else:
                self.send_cmd(CMD_STOP)

        elif self.current_state == STATE_REVERSING:
            if elapsed < self.reverse_time:
                self.send_cmd(CMD_BACKWARD_STRAIGHT)
            else:
                self.set_state(STATE_LOOK_L)

        elif self.current_state == STATE_LOOK_L:
            if elapsed < self.sensor_time:
                self.send_cmd(CMD_LEFT)
            else:
                self.set_state(STATE_SCAN_L)

        elif self.current_state == STATE_SCAN_L:
            self.send_cmd(CMD_STOP)
            if elapsed > 0.5:
                if not msg.front_valid or msg.proximity_front > self.wall_threshold:
                    self.chosen_turn_cmd = CMD_FORWARD_LEFT
                    self.set_state(STATE_DRIVE_OUT)
                else:
                    self.set_state(STATE_LOOK_R)

        elif self.current_state == STATE_LOOK_R:
            if elapsed < self.sensor_time:
                self.send_cmd(CMD_RIGHT)
            else:
                self.set_state(STATE_SCAN_R)

        elif self.current_state == STATE_SCAN_R:
            self.send_cmd(CMD_STOP)
            if elapsed > 0.5:
                if not msg.front_valid or msg.proximity_front > self.wall_threshold:
                    self.chosen_turn_cmd = CMD_FORWARD_RIGHT
                    self.set_state(STATE_DRIVE_OUT)
                else:
                    self.set_state(STATE_REV_CHECK)

        elif self.current_state == STATE_DRIVE_OUT:
            if elapsed < self.maneuver_time:
                self.send_cmd(self.chosen_turn_cmd)
            else:
                self.send_cmd(CMD_STOP)
                self.set_state(STATE_FORWARD)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
