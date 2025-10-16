#!/usr/bin/env python3
import sys
import termios
import tty
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from duman_interfaces.msg import DumanJoints

HELP_TEXT = """
===============================
  6-DOF Arm Joint Teleop (Toggle)
===============================
Controls:
  q : toggle Joint 1 (+ → - → stop)
  w : toggle Joint 2 (+ → - → stop)
  e : toggle Joint 3 (+ → - → stop)
  r : toggle Joint 4 (+ → - → stop)
  t : toggle Joint 5 (+ → - → stop)
  y : toggle Joint 6 (+ → - → stop)
  z : Stop all joints
  x : Exit
===============================
"""

# Function to read a single key (blocking)
def get_key():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class JointVelTeleop(Node):
    def __init__(self):
        super().__init__('teleop_joint_vel')
        self.publisher_ = self.create_publisher(DumanJoints, '/joint_vel', 10)

        # Parameters
        self.vel_mag = 40.0
        self.joint_vel = [0.0] * 6
        self.states = [0] * 6  # 0=stopped, 1=+vel, 2=-vel

        self.joint_names = [
            'hip', 'shoulder', 'elbow',
            'wrist1', 'wrist2', 'wrist3'
        ]

        self.key_mapping = {
            'q': 0, 'w': 1, 'e': 2,
            'r': 3, 't': 4, 'y': 5
        }

        self.get_logger().info("Started teleop_joint_vel node (toggle mode).")
        print(HELP_TEXT)

        # Timer to continuously publish joint velocities
        self.timer = self.create_timer(0.01, self.publish_joint_vel)

    def publish_joint_vel(self):
        msg = DumanJoints()
        msg.left_hip = self.joint_vel[0]
        msg.left_shoulder = self.joint_vel[1]
        msg.left_elbow = self.joint_vel[2] * 0.5
        msg.left_wrist1 = self.joint_vel[3]
        msg.left_wrist2 = self.joint_vel[4]
        msg.left_wrist3 = self.joint_vel[5]

        self.publisher_.publish(msg)

    def toggle_joint(self, joint_idx):
        # Cycle state: 0 -> + -> - -> 0
        if self.states[joint_idx] == 0:
            self.states[joint_idx] = 1
            self.joint_vel[joint_idx] = self.vel_mag
            state_text = "forward"
        elif self.states[joint_idx] == 1:
            self.states[joint_idx] = 2
            self.joint_vel[joint_idx] = -self.vel_mag
            state_text = "reverse"
        else:
            self.states[joint_idx] = 0
            self.joint_vel[joint_idx] = 0.0
            state_text = "stopped"

        self.get_logger().info(f"{self.joint_names[joint_idx]} → {state_text}")

    def process_key(self, key):
        if key in self.key_mapping:
            idx = self.key_mapping[key]
            self.toggle_joint(idx)
        elif key == 'z':
            self.joint_vel = [0.0] * 6
            self.states = [0] * 6
            self.get_logger().info("All joints stopped.")
        elif key in ['x', '\x03']:  # 'x' or Ctrl+C
            self.get_logger().info("Exiting teleop...")
            rclpy.shutdown()
            sys.exit(0)


def keyboard_thread(node):
    """Runs in a separate thread to read keyboard input."""
    try:
        while rclpy.ok():
            key = get_key()
            node.process_key(key)
    except KeyboardInterrupt:
        pass


def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = JointVelTeleop()

    # Create multi-threaded executor
    executor = MultiThreadedExecutor()

    # Add node to executor
    executor.add_node(node)

    # Start keyboard thread
    kb_thread = threading.Thread(target=keyboard_thread, args=(node,), daemon=True)
    kb_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
