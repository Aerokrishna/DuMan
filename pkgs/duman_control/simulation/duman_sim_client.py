#!/usr/bin/env python3
# Subscribes from /joint_states
# Converts to degrees
# Records one joint's angle vs time and plots it on KeyboardInterrupt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16
from duman_interfaces.msg import DumanJoints
import numpy as np
import time
from duman_interfaces.srv import GripState
from armos_interfaces.msg import ArmosJointControl
from armos_interfaces.srv import ArmosSetGripper

class DumanSimulationNode(Node):
    def __init__(self):
        super().__init__("duman_simulation")
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        self.left_joint_pub = self.create_publisher(ArmosJointControl, "/armos/left/joint_control", 10)
        self.right_joint_pub = self.create_publisher(ArmosJointControl, "/armos/right/joint_control", 10)
        self.left_gripper_client = self.create_client(
            ArmosSetGripper, "/armos/left/setgripper"
        )
        self.right_gripper_client = self.create_client(
            ArmosSetGripper, "/armos/right/setgripper"
        )
        self.create_service(GripState, "/duman/grip_state", self.grip_state_callback)

        # Initialize joint data
        self.joint_angles = np.zeros(12)
        self.joint_velocity = np.zeros(12)

        self.right = True
        self.left = True

    def grip_state_callback(self, request, response):
        gripper_request = ArmosSetGripper.Request()
        gripper_request.position = float(not(request.grip_state))

        if request.arm == 0:
            self.right_gripper_client.call_async(gripper_request)
        if request.arm == 1:
            self.left_gripper_client.call_async(gripper_request)

        response.success = True
        response.message = "Gripper request sent to simulation"
        return response
        
    def joint_state_callback(self, msg: JointState):
        # self.get_logger().info("joint state callback sim node")
        joint_angles = np.array([
            msg.position[9], msg.position[2], msg.position[7],
            msg.position[4],  msg.position[0], msg.position[1],
            msg.position[3], msg.position[5], msg.position[6],
             msg.position[8], msg.position[10], msg.position[11]
        ])

        # Convert to degrees (integers)
        # self.joint_angles = np.rad2deg(joint_angles)
        joint_msg = ArmosJointControl()
        joint_msg.mode = 0
        if self.left:
            left = joint_angles[6:]
            joint_msg.joint1 = left[0]
            joint_msg.joint2 = left[1]
            joint_msg.joint3 = left[2]
            joint_msg.joint4 = left[3]
            joint_msg.joint5 = left[4]
            joint_msg.joint6 = left[5]

            self.left_joint_pub.publish(joint_msg)

            # right the left joints
            # blitz_interfaces["joint_angles_left"].data = joint_angles[6:]
            # self.blitz_left.blitz_write(id=blitz_interfaces["joint_angles_left"].id)

        if self.right:
            right = joint_angles[:6]
            joint_msg.joint1 = right[0]
            joint_msg.joint2 = right[1]
            joint_msg.joint3 = right[2]
            joint_msg.joint4 = right[3]
            joint_msg.joint5 = right[4]
            joint_msg.joint6 = right[5]

            self.right_joint_pub.publish(joint_msg)
            # write the right joints
            # blitz_interfaces["joint_angles_right"].data = self.joint_angles[:6]
            # self.blitz_right.blitz_write(id=blitz_interfaces["joint_angles_right"].id)

        # self.get_logger().info(f"WRITING JOINTs : {self.joint_angles}")

def main(args=None):
    rclpy.init(args=args)
    node = DumanSimulationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Plotting data...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
