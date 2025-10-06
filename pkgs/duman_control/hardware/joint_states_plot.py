#!/usr/bin/env python3
# Subscribes from /joint_states
# Converts to degrees
# Records one joint's angle vs time and plots it on KeyboardInterrupt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
import time

class DumanHardwareNode(Node):
    def __init__(self):
        super().__init__("duman_hardware")
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        # Initialize joint data
        self.joint_angles = np.zeros(12)
        self.joint_velocity = np.zeros(12)

        self.time_data = []
        self.angle_data = []
        self.start_time = time.time()

        # Choose which joint index to plot (0–11)
        self.target_joint_index = 2

    def joint_state_callback(self, msg: JointState):
        joint_angles = np.array([
            msg.position[9], msg.position[2], msg.position[7],
            msg.position[4], msg.position[0], msg.position[1],
            msg.position[3], msg.position[5], msg.position[6],
            msg.position[8], msg.position[10], msg.position[11]
        ])

        joint_vel = np.array([
            msg.velocity[9], msg.velocity[2], msg.velocity[7],
            msg.velocity[4], msg.velocity[0], msg.velocity[1],
            msg.velocity[3], msg.velocity[5], msg.velocity[6],
            msg.velocity[8], msg.velocity[10], msg.velocity[11]
        ])

        # Convert to degrees (integers)
        self.joint_angles = np.rad2deg(joint_angles).astype(np.float32)
        self.joint_velocity = np.rad2deg(joint_vel).astype(np.float32)


        # Record time and target joint angle
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.angle_data.append(self.joint_velocity[self.target_joint_index])

        self.get_logger().info(f"JOINT ANGLES IN DEG : {self.joint_velocity}")

    def plot_joint_angle(self):
        plt.figure(figsize=(8, 4))
        plt.plot(self.time_data, self.angle_data, '-b')
        plt.title(f"Joint {self.target_joint_index} Angle vs Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (deg)")
        plt.grid(True)
        plt.tight_layout()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = DumanHardwareNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Plotting data...")
        node.plot_joint_angle()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
