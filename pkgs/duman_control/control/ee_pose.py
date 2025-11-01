#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymoveit2 import MoveIt2
from pymoveit2.robots import duman_left, duman_right
from duman_interfaces.msg import DumanPose
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from rclpy.executors import MultiThreadedExecutor

class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')
        # Subscribe to the /joint_states topic
        self.subscription = self.create_subscription(JointState, '/joint_states', self.feedback_joint_angle,10)
        
        self.right_joints = []
        self.left_joints = []

        self.left_arm_moveit = MoveIt2(
            node=self,
            joint_names=duman_left.joint_names(),
            base_link_name=duman_left.base_link_name(),
            end_effector_name=duman_left.end_effector_name(),
            group_name=duman_left.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup(),
            follow_joint_trajectory_action_name="duman_left_controller/follow_joint_trajectory",

        )

        self.right_arm_moveit = MoveIt2(
            node=self,
            joint_names=duman_right.joint_names(),
            base_link_name=duman_right.base_link_name(),
            end_effector_name=duman_right.end_effector_name(),
            group_name=duman_right.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup(),
            follow_joint_trajectory_action_name="duman_right_controller/follow_joint_trajectory",

        )

        self.pose_pub = self.create_publisher(DumanPose, "/duman/pose", 10)

        self.create_timer(0.1, self.pose_pub_cb)
        self.get_logger().info("PUBLISHING")


    def feedback_joint_angle(self, msg: JointState):

        self.left_joints = [msg.position[3], msg.position[5],msg.position[6],
                            msg.position[8], msg.position[10],msg.position[11]]
        
        self.right_joints = [msg.position[9], msg.position[2],msg.position[7],
                        msg.position[4], msg.position[0],msg.position[1]]
    
    def pose_pub_cb(self):
        self.get_logger().info("PUBLISHING")

        pose_msg = DumanPose()

        joints = JointState()
        joints.name = duman_left.joint_names()
        joints.position = self.left_joints
        
        ee_pose = self.left_arm_moveit.compute_fk(joints)[0]
        eul = euler_from_quaternion([ee_pose.pose.orientation.x, ee_pose.pose.orientation.y, ee_pose.pose.orientation.z, ee_pose.pose.orientation.w])


        pose_msg.left_pos_x = ee_pose.pose.position.x
        pose_msg.left_pos_y = ee_pose.pose.position.y
        pose_msg.left_pos_z = ee_pose.pose.position.z
        pose_msg.left_or_x = abs(eul[0])
        pose_msg.left_or_y = abs(eul[1])
        pose_msg.left_or_z = abs(eul[2])

        joints = JointState()
        joints.name = duman_right.joint_names()
        joints.position = self.right_joints
        
        ee_pose = self.right_arm_moveit.compute_fk(joints)[0]
        eul = euler_from_quaternion([ee_pose.pose.orientation.x, ee_pose.pose.orientation.y, ee_pose.pose.orientation.z, ee_pose.pose.orientation.w])

        pose_msg.right_pos_x = ee_pose.pose.position.x
        pose_msg.right_pos_y = ee_pose.pose.position.y
        pose_msg.right_pos_z = ee_pose.pose.position.z
        pose_msg.right_or_x = abs(eul[0])
        pose_msg.right_or_y = abs(eul[1])
        pose_msg.right_or_z = abs(eul[2])

        self.pose_pub.publish(pose_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = JointStateListener()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
