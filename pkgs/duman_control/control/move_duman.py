#!/usr/bin/env python3

# takes joint and pose goals of right and left arm
# sends request to the move group async
# checks if goal is active and then rejects goal request
# able to cancel goal
# will take feedback from real robot and send it 
# tells if plan is success in form of feedback, and that execution started
# result means the execution was finished

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
import time
from duman_interfaces.action import DumanGoal
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
from pymoveit2 import MoveIt2
from pymoveit2.robots import duman_right, duman_left
import numpy as np
from duman_interfaces.msg import DumanJoints, DumanPose

class ArmMonitor():
    joint_angles = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    # ee_pose = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    goal_reached = False

class CountUntilServerNode(Node):
    def __init__(self):
        super().__init__("count_until_server")
        self.goal_handle_ : ServerGoalHandle = None

        # to prevent multiple threads from accessing a shared resource simultaneously. 
        # It ensures that only one thread can hold the lock at a time
        self.goal_lock_ = threading.Lock()
        self.goal_queue_ = []

        self.duman_left = ArmMonitor()
        self.duman_right = ArmMonitor()

        # INITIALIZE SERVER
        self.joint_goal_server_ = ActionServer(
            self, 
            DumanGoal,  
            "/duman/goal",
            goal_callback=self.goal_callback, 
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback, 
            execute_callback=self.execute_callback, 
            callback_group=ReentrantCallbackGroup()) 
        
        # RIGHT ARM
        self.right_arm_moveit = MoveIt2(
            node=self,
            joint_names=duman_right.joint_names(),
            base_link_name=duman_right.base_link_name(),
            end_effector_name=duman_right.end_effector_name(),
            group_name=duman_right.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup(),
        )
         
        # LEFT ARM
        self.left_arm_moveit = MoveIt2(
            node=self,
            joint_names=duman_left.joint_names(),
            base_link_name=duman_left.base_link_name(),
            end_effector_name=duman_left.end_effector_name(),
            group_name=duman_left.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup(),
        )
        
        self.get_logger().info("move duman server started")

    def goal_callback(self, goal_request: DumanGoal.Goal):
        
        # reject the goal if a goal is already executing
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().error("GOAL ACTIVE...rejecting new goal")
                return GoalResponse.REJECT
        
        self.get_logger().info("GOAL ACCEPTED!")
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle): # goal handle is going to be cancelled as client will cancel a particular goal
        self.get_logger().info("cancel request received...")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle : ServerGoalHandle):

        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        
        # JOINT GOAL
        if goal_handle.request.goal_type == 0:
            joint_goal = [goal_handle.request.hip,
                        goal_handle.request.shoulder,
                        goal_handle.request.elbow,
                        goal_handle.request.wrist1,
                        goal_handle.request.wrist2,
                        goal_handle.request.wrist3]
            
            # print(self.left_arm_moveit.compute_fk(joint_state=joint_goal))

            if goal_handle.request.arm:
                # left arm
                self.left_arm_moveit.move_to_configuration(joint_goal)
            
            else:
                self.right_arm_moveit.move_to_configuration(joint_goal)

        # POSE GOAL
        else:
            pass
        
        result = DumanGoal.Result()
        result.success = True
        result.message = "Succeeded"

        goal_handle.succeed()

        return result

    def feedback_joint_angle(self, joint_angle : DumanJoints):

        self.duman_left.joint_angles = np.array[joint_angle.left_hip, joint_angle.left_shoulder,joint_angle.left_elbow,
                                                joint_angle.left_wrist1, joint_angle.left_wrist2,joint_angle.left_wrist3]
        
        self.duman_right.joint_angles = np.array[joint_angle.right_hip, joint_angle.right_shoulder,joint_angle.right_elbow,
                                                joint_angle.right_wrist1, joint_angle.right_wrist2,joint_angle.right_wrist3]

    # def feedback_ee_pose(self, ee_pose : DumanPose):

    #     self.duman_left.ee_pose = np.array[ee_pose.left_pos_x, ee_pose.left_pos_y,ee_pose.left_pos_z,
    #                                     ee_pose.left_or_x, ee_pose.left_or_y,ee_pose.left_or_z]
        
    #     self.duman_right.ee_pose = np.array[ee_pose.right_pos_x, ee_pose.right_pos_y,ee_pose.right_pos_z,
    #                                     ee_pose.right_or_x, ee_pose.right_or_y,ee_pose.right_or_z]
        
    def goal_checker(self, arm, target_joint_angles : np.ndarray):

        threshold = 0.3

        if arm == 0:
            diff = np.abs(self.duman_right.joint_angles - target_joint_angles)  
            return np.all(diff < threshold)

        else:
            diff = np.abs(self.duman_left.joint_angles - target_joint_angles)  
            return np.all(diff < threshold)
        
def main(args=None):
    rclpy.init(args=args)
    node = CountUntilServerNode()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__=="__main__":
    main()


    # for i in range(target_number):

    #         # if goal handle is not active 
    #         if not goal_handle.is_active:
    #             result.reached_number = counter
    #             return result
            
    #         # if goal handle is in cancel state
    #         if goal_handle.is_cancel_requested:
    #             self.get_logger().info("Cancelling Goal")
    #             goal_handle.canceled() # cancel the goal, similarly we can set it as aborted or succeeded
    #             result.reached_number = counter 
    #             return result # return whatever result
 
    #         counter += 1
    #         self.get_logger().info(str(counter))

    #         # send feedback in every loop
    #         feedback.current_number = counter
    #         goal_handle.publish_feedback(feedback)
    #         time.sleep(period) # simulating the the time taken by robot to execute the action
        