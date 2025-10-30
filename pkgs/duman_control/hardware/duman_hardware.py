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
from duman_interfaces.srv import GripState

class DumanHardwareNode(Node):
    def __init__(self):
        super().__init__("duman_hardware")
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.create_subscription(DumanJoints, "/joint_vel", self.joint_vel_callback, 10)

        self.create_service(GripState, "/duman/grip_state", self.grip_control)
        # Initialize joint data
        self.joint_angles = np.zeros(12)
        self.joint_velocity = np.zeros(12)

        # self.blitz_left = Blitz()
        self.blitz_right = Blitz()

        self.right_grip_state = False
        self.left_grip_state = True

        self.create_timer(0.005, self.joint_state_feedback)

    def grip_control(self, request : GripState.Request, response : GripState.Response):

        print('REQUEST AYA HAI ', request.arm)

        if request.arm==True:
            self.left_grip_state = request.grip_state
            self.get_logger().info(f"GRIPER LEFT ARM REQUESTED")

            for i in range(5):
                blitz_interfaces["grip_state_left"].data = [int(self.left_grip_state)]
                self.blitz_right.blitz_write(id=blitz_interfaces["grip_state_left"].id)

        else:
            self.left_grip_state = request.grip_state
            self.get_logger().info(f"GRIPER RIGHT ARM REQUESTED")
            
            for i in range(5):
                blitz_interfaces["grip_state_right"].data = [int(self.left_grip_state)]
                self.blitz_right.blitz_write(id=blitz_interfaces["grip_state_right"].id)
        
        response.message = "successfully controlled gripper"
        response.success = True

        return response

    def joint_state_callback(self, msg: JointState):
        joint_angles = np.array([
            msg.position[9], msg.position[2], msg.position[7],
            (np.pi/2)-msg.position[4], msg.position[0]+(np.pi/2), msg.position[1]+(np.pi/2),
            msg.position[3], msg.position[5], msg.position[6],
            msg.position[8]+(np.pi/2), msg.position[10]+(np.pi/2), msg.position[11]+(np.pi/2)
        ])

        # Convert to degrees (integers)
        self.joint_angles = np.rad2deg(joint_angles)
        
        # right the left joints
        # blitz_interfaces["joint_angles_left"].data = self.joint_angles[6:]
        # self.blitz_left.blitz_write(id=blitz_interfaces["joint_angles_left"].id)

        # # write the right joints
        blitz_interfaces["joint_angles_right"].data = self.joint_angles[:6]
        self.blitz_right.blitz_write(id=blitz_interfaces["joint_angles_right"].id)

        # self.get_logger().info(f"WRITING JOINTs : {self.joint_angles}")

    def joint_vel_callback(self, msg: DumanJoints):
        
        joint_vel = np.array([msg.right_hip, msg.right_shoulder, msg.right_elbow, msg.right_wrist1, msg.right_wrist2, msg.right_wrist3,
                              msg.left_hip, msg.left_shoulder, msg.left_elbow, msg.left_wrist1, msg.left_wrist2, msg.left_wrist3])

        # blitz_interfaces["joint_vel_left"].data = joint_vel[6:]
        # self.blitz_left.blitz_write(id=blitz_interfaces["joint_vel_left"].id)

        blitz_interfaces["joint_vel_right"].data = joint_vel[:6]
        self.blitz_right.blitz_write(id=blitz_interfaces["joint_vel_right"].id)

        # self.get_logger().info(f"WRITING Joint Velocities : {joint_vel}")

    def joint_state_feedback(self):

        self.blitz_right.blitz_read()

        # joint_fb = blitz_interfaces["joint_angles_left_feedback"].data
        # self.get_logger().info(f"CURRENT LEFT JOINTs : {joint_fb}")

        joint_fb = blitz_interfaces["joint_angles_right_feedback"].data
        # self.get_logger().info(f"CURRENT RIGHT JOINTs : {joint_fb}")

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
