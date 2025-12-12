#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from duman_interfaces.action import DumanGoal
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
from pymoveit2 import MoveIt2
from pymoveit2.robots import duman_left
import numpy as np
from duman_interfaces.msg import DumanPose
from sensor_msgs.msg import JointState
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
from objects import objects_left
import time

class MoveDumanLeft(Node):
    def __init__(self):
        super().__init__("move_duman_left")
        self.goal_handle_ : ServerGoalHandle = None

        # to prevent multiple threads from accessing a shared resource simultaneously. 
        # It ensures that only one thread can hold the lock at a time
        self.goal_lock_ = threading.Lock()
        self.goal_queue_ = []

        self.current_joint_angles = np.zeros(6, dtype=np.float32)

        self.objects_left = objects_left  

        # INITIALIZE SERVER
        self.joint_goal_server_ = ActionServer(
            self, 
            DumanGoal,  
            "/duman/goal_left",
            goal_callback=self.goal_callback, 
            cancel_callback=self.cancel_callback, 
            execute_callback=self.execute_callback, 
            callback_group=ReentrantCallbackGroup()) 
        
        # left ARM
        self.left_arm_moveit = MoveIt2(
            node=self,
            joint_names=duman_left.joint_names(),
            base_link_name=duman_left.base_link_name(),
            end_effector_name=duman_left.end_effector_name(),
            group_name=duman_left.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup(),
            follow_joint_trajectory_action_name="duman_left_controller/follow_joint_trajectory",

        )
        
        # self.create_subscription(DumanJoints, "/joint_states", self.feedback_joint_angle, 10)
        self.create_subscription(JointState, "/joint_states", self.feedback_joint_angle, 10)
        self.create_subscription(DumanPose, "/duman/pose", self.feedback_pose, 10)

        self.in_zone = False
        self.zone_time = 0.0

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.joint_goal = np.zeros(6)
        self.ik_pose_goal = np.zeros(6)
        self.pose_goal = np.zeros(6)
        self.current_pose = np.zeros(6)
        self.plan_pose_goal = None

        self.last_time = time.monotonic()
            
        self.get_logger().info("move duman server started")

    def goal_callback(self, goal_request: DumanGoal.Goal):
        
        # reject the goal if a goal is already executing
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().error("GOAL ACTIVE...rejecting new goal")
                return GoalResponse.REJECT
        
        if goal_request.goal_type == 0:
            self.joint_goal = [goal_request.hip,
                        goal_request.shoulder,
                        goal_request.elbow,
                        goal_request.wrist1,
                        goal_request.wrist2,
                        goal_request.wrist3]
            
            if not duman_left.joint_goal_valid(self.joint_goal):
                self.get_logger().error(f"Joint goal outside joint limits, Rejecting GOAL {self.joint_goal}")
                return GoalResponse.REJECT

            self.get_logger().info("JOINT GOAL ACCEPTED!")

        elif goal_request.goal_type == 1:
            # pose goal
            obj = goal_request.object_id

            if obj == "default":

                position = [goal_request.x, goal_request.y, goal_request.z]
                quat = list(quaternion_from_euler(goal_request.orx, goal_request.ory, goal_request.orz, "rxyz"))

                self.pose_goal = [goal_request.x, goal_request.y, goal_request.z, goal_request.orx, goal_request.ory, goal_request.orz]
            else:
                position = [self.objects_left[obj][0], self.objects_left[obj][1], self.objects_left[obj][2]]
                quat = list(quaternion_from_euler(self.objects_left[obj][3], self.objects_left[obj][4], self.objects_left[obj][5], "rxyz"))
                self.pose_goal = [self.objects_left[obj][0], self.objects_left[obj][1], self.objects_left[obj][2],
                                  self.objects_left[obj][3], self.objects_left[obj][4], self.objects_left[obj][5]]
                
            ik = self.left_arm_moveit.compute_ik(position=position, quat_xyzw=quat)

            if ik is None:
                self.get_logger().error("Inverse Kinematics Failed, Rejecting GOAL")
                return GoalResponse.REJECT

            self.ik_pose_goal = np.array(ik.position[:6])

            self.left_arm_moveit.set_position_goal(position=position, frame_id=duman_left.base_link_name(), target_link=duman_left.end_effector_name())
            self.left_arm_moveit.set_orientation_goal(quat_xyzw=quat, frame_id=duman_left.base_link_name(), target_link=duman_left.end_effector_name())

            self.plan_pose_goal = self.left_arm_moveit.plan()

            if self.plan_pose_goal is None:
                self.get_logger().error("Planning Failed, Rejecting GOAL")
                return GoalResponse.REJECT

            self.get_logger().info("POSE GOAL ACCEPTED!")

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle): # goal handle is going to be cancelled as client will cancel a particular goal
        self.get_logger().info("cancel request received...")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle : ServerGoalHandle):

        with self.goal_lock_:
            self.goal_handle_ = goal_handle
        
        # JOINT GOAL
        if goal_handle.request.goal_type == 0:
            
            self.left_arm_moveit.move_to_configuration(self.joint_goal)

            while not self.goal_checker(np.array(self.joint_goal)):
                pass

            result = DumanGoal.Result()
            result.success = True
            result.message = "Joint Goal Succeeded"
            goal_handle.succeed()

                # self.get_logger().info(f"EXECUTING JOINT GOAL for duman left")
            
        # POSE GOAL
        else:
            self.left_arm_moveit.execute(self.plan_pose_goal)
            self.get_logger().info(f"{self.ik_pose_goal}")

            while not self.goal_checker(np.array(self.pose_goal), thresh=0.03, joint=False):
                pass
        
            result = DumanGoal.Result()
            result.success = True
            result.message = "Pose Goal Succeeded"

            goal_handle.succeed()
        
        self.get_logger().info("GOAL FINISH RETURNING SUCCESS!")

        return result
    
    def feedback_joint_angle(self, msg : JointState):
            
        self.current_joint_angles = np.array([msg.position[3], msg.position[5],msg.position[6],
                                                msg.position[8], msg.position[10],msg.position[11]])

    def feedback_pose(self, msg:DumanPose):

        self.current_pose = np.array([msg.left_pos_x, msg.left_pos_y, msg.left_pos_z,
                                      msg.left_or_x, msg.left_or_y, msg.left_or_z])
        
    def goal_checker(self, target: np.ndarray, thresh=0.05, joint=True):

        if joint:
            diff = np.abs(self.current_joint_angles - target)
        else:
            diff = np.abs(self.current_pose[:3] - target[:3])

        inside = np.all(diff < thresh)

        if inside:
            self.in_zone = True
            # Trigger after 3 seconds inside zone
            if self.zone_time >= 1.5:
                return True
            else:
                return False
        else:
            self.in_zone = False
            return False
    
    def initialize_arm(self):
        self.left_arm_moveit.move_to_configuration([0.05, -0.75, 2.0, 0.0,0.0,0.0])
        # self.left_arm_moveit.wait_until_executed()
        self.get_logger().info("ARMS INITIALIZED")

    def timer_callback(self):
        if self.in_zone:
            self.zone_time += 0.1
        else:
            self.zone_time = 0.0  # reset when outside

def main(args=None):
    rclpy.init(args=args)
    node = MoveDumanLeft()

    time.sleep(4.0)
    node.initialize_arm()

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
        