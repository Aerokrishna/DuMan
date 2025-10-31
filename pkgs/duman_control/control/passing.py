#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from duman_interfaces.action import DumanGoal
from duman_interfaces.srv import DumanPass, GripState

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import numpy as np
from duman_interfaces.msg import DumanJoints
from sensor_msgs.msg import JointState
from tf_transformations import quaternion_from_euler
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
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

        # create action client
        self.duman_left_goal_client_ = ActionClient(self, DumanGoal, "/duman/goal_left", callback_group=ReentrantCallbackGroup())
        self.duman_right_goal_client_ = ActionClient(self, DumanGoal, "/duman/goal_right", callback_group=ReentrantCallbackGroup())

        self.duman_grip_client = self.create_client(GripState, "/duman/grip_state", callback_group=ReentrantCallbackGroup())

        self.arm_done = False
        self.state = 0
        self.goal_sent = False

        self.right_transfer_position = [-0.1, -0.2, 0.25, -1.57, 1.57, 0.0]
        self.left_transfer_position = [0.1, -0.2, 0.25, 1.57, -1.57, 0.0]

        self.right_grasp_position = [-0.03, -0.2, 0.25, -1.57, 1.57, 0.0]
        self.left_grasp_position = [0.03, -0.2, 0.25, 1.57, -1.57, 0.0]

        self.get_logger().info("duman passing server started")

        self.create_service(DumanPass, "/duman/pass", self.pass_callback)

        self.last_time = time.monotonic()
        self.cnt = 0

        self.get_logger().info("waiting for server....")
        self.duman_right_goal_client_.wait_for_server() # you can provide a timer to wait for the server inside
        self.duman_left_goal_client_.wait_for_server() # you can provide a timer to wait for the server inside
        self.get_logger().info("server found!")

    def delay_(self, period):
        current_time = time.monotonic()
        if current_time - self.last_time >= period:
            self.last_time = current_time
            return True
        return False
    
    def pass_callback(self, request : DumanPass.Request, response : DumanPass.Response):
        self.state = 1
        # self.get_logger().info("PASSING REQUEST RECEIVED")

        while True:
            time.sleep(0.1)
            # if self.delay_(0.1):
            self.get_logger().info(f"{self.state}")
            if self.state == 1:
                self.get_logger().info(f"Moving to pass")

                if not self.goal_sent:
                    self.arm_done = False

                    self.send_goal(arm=False, goal_type=True, target=self.right_transfer_position)
                    self.send_goal(arm=True, goal_type=True, target=self.left_transfer_position)
                    self.goal_sent = True

            elif self.state == 2 and self.delay_(2.0):
                self.get_logger().info(f"Opening grip")

                if not self.goal_sent:
                    self.send_grip_cmd(arm=request.to_arm, grip_state=False)
                    self.goal_sent = True

            elif self.state == 3 and self.delay_(2.0):
                self.get_logger().info(f"moving close")

                if not self.goal_sent:
                    self.arm_done = False
                    
                    self.send_goal(arm=False, goal_type=True, target=self.right_grasp_position)
                    self.send_goal(arm=True, goal_type=True, target=self.left_grasp_position)

                    self.goal_sent = True

            elif self.state == 4 and self.delay_(1.0):
                self.get_logger().info(f"grip the arm")

                if not self.goal_sent:

                    self.send_grip_cmd(arm=request.to_arm, grip_state=True)
                    self.goal_sent = True
            
            elif self.state == 5 and self.delay_(2.0):
                self.get_logger().info(f"ungrip the arm")

                if not self.goal_sent:

                    self.send_grip_cmd(arm=(not request.to_arm), grip_state=True)

                    self.goal_sent = True

            
            elif self.state == 6 and self.delay_(2.0):
                self.get_logger().info(f"moving away")

                if not self.goal_sent:
                    self.arm_done = False
                    
                    self.send_goal(arm=False, goal_type=True, target=self.right_transfer_position)
                    self.send_goal(arm=True, goal_type=True, target=self.left_transfer_position)

                    self.goal_sent = True


            if self.state == 7:
                self.get_logger().info(f"IDLING")

                self.state = 0
                break
      
        response.message = "successfully controlled gripper"
        response.success = True

        return response
    
    def send_goal(self, arm, goal_type, target):

        # Define your goal as your custom action
        goal = DumanGoal.Goal()

        goal.arm = arm #right arm
        goal.object_id = "default"

        if goal_type == 0:
            goal.goal_type = goal_type #joint goal
            goal.hip = target[0]
            goal.shoulder = target[1]
            goal.elbow = target[2]
            goal.wrist1 = target[3]
            goal.wrist2 = target[4]
            goal.wrist3 = target[5]

            # self.get_logger().info("JOINT Goal sending")
        
        else:
            goal.goal_type = goal_type #joint goal
            goal.x = target[0]
            goal.y = target[1]
            goal.z = target[2]
            goal.orx = target[3]
            goal.ory = target[4]
            goal.orz = target[5]

            # self.get_logger().info("POSE Goal sending")

        if arm==1:
            self.duman_left_goal_client_.send_goal_async(goal).add_done_callback(self.goal_response_callback) 
        
        else:
            self.duman_right_goal_client_.send_goal_async(goal).add_done_callback(self.goal_response_callback) 

    def send_grip_cmd(self, arm, grip_state):
        # Create a request for the ArucoSW service, to get the pick and drop coordinates.

        req = GripState.Request()
        req.grip_state = grip_state
        req.arm = arm

        if arm==False:
            self.get_logger().info("GRIPPER RIGHT!")

            # Call the service asynchronously
            future = self.duman_grip_client.call_async(req)
            future.add_done_callback(self.grip_result_callback)
        
        else :
            # Call the service asynchronously
            self.get_logger().info("GRIPPER LEFT!")

            future = self.duman_grip_client.call_async(req)
            future.add_done_callback(self.grip_result_callback)
    
    def grip_result_callback(self, future):
        try:
            if future.result().success:
                self.state += 1
                self.goal_sent = False

            # self.get_logger().info(f'Service response: {self.response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def goal_response_callback(self, future):
        # callback to see if goal was accpeted
        self.goal_handle_: ClientGoalHandle = future.result()

        if self.goal_handle_.accepted:
            # self.get_logger().info("GOAL ACCEPTED!")

            # add a callback which runs when a result is received
            self.goal_handle_.get_result_async().add_done_callback(self.motion_result_callback) # call the future callback
        else:
            self.get_logger().warn("GOAL REJECTED")

    # a callback to signify completion of an arm motion task
    # the motion result will include which arm has completed the motion task
    def motion_result_callback(self,future):
        status = future.result().status
        result = future.result().result # is the reached number interface made in actions

        if status == GoalStatus.STATUS_SUCCEEDED:
            if self.arm_done:
                self.state+=1
                self.goal_sent = False

            self.arm_done = True
            
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("ABORTED")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error("CANCELLED")

        # self.get_logger().info(f"Result : {result.message} {result.success}")  #+ str(result.reached_number)
    
    def cancel_goal(self):
        self.get_logger().info("Sending cancel request")
        self.goal_handle_.cancel_goal_async()

def main(args=None):
    rclpy.init(args=args)
    node = MoveDumanLeft()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__=="__main__":
    main()

