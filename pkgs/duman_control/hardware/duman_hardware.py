#!/usr/bin/env python3
# Subscribes from /joint_states
# Converts to degrees
# Records one joint's angle vs time and plots it on KeyboardInterrupt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from duman_interfaces.msg import DumanJoints
import numpy as np
import time
from blitz import blitz_interfaces, Blitz

class DumanHardwareNode(Node):
    def __init__(self):
        super().__init__("duman_hardware")
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.create_subscription(DumanJoints, "/joint_vel", self.joint_vel_callback, 10)

        # Initialize joint data
        self.joint_angles = np.zeros(12)
        self.joint_velocity = np.zeros(12)

        self.blitz = Blitz()

        self.create_timer(0.005, self.joint_state_feedback)

    def joint_state_callback(self, msg: JointState):
        joint_angles = np.array([
            msg.position[9], msg.position[2], msg.position[7],
            msg.position[4]+(np.pi/2), msg.position[0]+(np.pi/2), msg.position[1]+(np.pi/2),
            msg.position[3], msg.position[5], msg.position[6],
            msg.position[8]+(np.pi/2), msg.position[10]+(np.pi/2), msg.position[11]+(np.pi/2)
        ])

        # Convert to degrees (integers)
        self.joint_angles = np.rad2deg(joint_angles)
        
        blitz_interfaces["joint_angles_left"].data = self.joint_angles[6:]
        self.blitz.blitz_write(id=blitz_interfaces["joint_angles_left"].id)

        # self.get_logger().info(f"WRITING JOINTs : {self.joint_angles}")

    def joint_vel_callback(self, msg: DumanJoints):
        
        joint_vel = np.array([msg.left_hip, msg.left_shoulder, msg.left_elbow, msg.left_wrist1, msg.left_wrist2, msg.left_wrist3])

        blitz_interfaces["joint_vel_left"].data = joint_vel
        self.blitz.blitz_write(id=blitz_interfaces["joint_vel_left"].id)

        self.get_logger().info(f"WRITING Joint Velocities : {joint_vel}")

    def joint_state_feedback(self):

        self.blitz.blitz_read()
        joint_fb = blitz_interfaces["joint_angles_left_feedback"].data
        self.get_logger().info(f"CURRENT JOINTs : {joint_fb} {self.joint_angles[6]}")


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
