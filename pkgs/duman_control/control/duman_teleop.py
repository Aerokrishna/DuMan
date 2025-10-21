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
=============================================
  6-DOF Arm Joint Teleop (Separate + / - Keys)
=============================================
Controls:
  Joint 1: q (+) / a (-)
  Joint 2: w (+) / s (-)
  Joint 3: e (+) / d (-)
  Joint 4: r (+) / f (-)
  Joint 5: t (+) / g (-)
  Joint 6: y (+) / h (-)

  SPACE : Stop all joints
  x     : Exit
=============================================
"""

def get_key():
    """Reads one key press without requiring Enter."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class JointVelTeleop(Node):
    def __init__(self):
        super().__init__('teleop_joint_vel')
        self.publisher_ = self.create_publisher(DumanJoints, '/joint_vel', 10)

        # Parameters
        self.vel_mag = 30.0
        self.joint_vel = [0.0] * 6

        self.joint_names = [
            'right_hip', 'right_shoulder', 'right_elbow',
            'left_hip', 'left_shoulder', 'left_elbow'
        ]

        # Key mapping for + and - direction
        self.key_mapping = {
            'q': (0, +1), 'a': (0, -1),
            'w': (1, +1), 's': (1, -1),
            'e': (2, +1), 'd': (2, -1),
            'r': (3, +1), 'f': (3, -1),
            't': (4, +1), 'g': (4, -1),
            'y': (5, +1), 'h': (5, -1),
        }

        self.get_logger().info("Started teleop_joint_vel node (separate + / - keys).")
        print(HELP_TEXT)

        # Timer to continuously publish joint velocities
        self.timer = self.create_timer(0.01, self.publish_joint_vel)

    def publish_joint_vel(self):
        msg = DumanJoints()

        msg.right_hip = self.joint_vel[0]
        msg.right_shoulder = self.joint_vel[1]
        msg.right_elbow = self.joint_vel[2] * 0.5  # scaled elbow speed
        msg.left_hip = self.joint_vel[3]
        msg.left_shoulder = self.joint_vel[4]
        msg.left_elbow = self.joint_vel[5] * 0.5

        self.publisher_.publish(msg)

    def process_key(self, key):
        # + or - joint control
        if key in self.key_mapping:
            idx, direction = self.key_mapping[key]
            self.joint_vel[idx] = direction * self.vel_mag
            state = "forward" if direction > 0 else "reverse"
            self.get_logger().info(f"{self.joint_names[idx]} moving {state}")

        # Stop all joints
        elif key == ' ':
            self.joint_vel = [0.0] * 6
            self.get_logger().info("All joints stopped.")

        # Exit
        elif key in ['x', '\x03']:
            self.get_logger().info("Exiting teleop...")
            rclpy.shutdown()
            sys.exit(0)


def keyboard_thread(node):
    """Runs keyboard reading in a separate thread."""
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

    executor = MultiThreadedExecutor()
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
