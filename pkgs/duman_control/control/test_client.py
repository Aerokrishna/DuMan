#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from duman_interfaces.action import DumanGoal
import time

class DumanGoalClient(Node):
    def __init__(self):
        super().__init__("count_until_server")

        # create action client
        self.duman_left_goal_client_ = ActionClient(self, DumanGoal, "/duman/goal_left")
        self.duman_right_goal_client_ = ActionClient(self, DumanGoal, "/duman/goal_right")


        self.get_logger().info("waiting for server....")
        self.duman_right_goal_client_.wait_for_server() # you can provide a timer to wait for the server inside
        # self.duman_left_goal_client_.wait_for_server() # you can provide a timer to wait for the server inside
        self.get_logger().info("server found!")

        self.ready_right = [-0.3, 0.5, -1.57, 0.0, -0.5, 0.0]
        self.ready_left = [0.3, -0.5, 1.57, 0.0, 0.5, 0.0]

    def send_goal(self, arm, goal_type, target):

        # Define your goal as your custom action
        goal = DumanGoal.Goal()

        goal.arm = arm #right arm

        if goal_type == 0:
            goal.goal_type = goal_type #joint goal
            goal.hip = target[0]
            goal.shoulder = target[1]
            goal.elbow = target[2]
            goal.wrist1 = target[3]
            goal.wrist2 = target[4]
            goal.wrist3 = target[5]

            self.get_logger().info("JOINT Goal sending")
        
        else:
            goal.goal_type = goal_type #joint goal
            goal.x = target[0]
            goal.y = target[1]
            goal.z = target[2]
            goal.orx = target[3]
            goal.ory = target[4]
            goal.orz = target[5]

            self.get_logger().info("POSE Goal sending")

        if arm==1:
            self.duman_left_goal_client_.send_goal_async(goal).add_done_callback(self.goal_response_callback) 
        
        else:
            self.duman_right_goal_client_.send_goal_async(goal).add_done_callback(self.goal_response_callback) 

    def goal_response_callback(self, future):
        # callback to see if goal was accpeted
        self.goal_handle_: ClientGoalHandle = future.result()

        if self.goal_handle_.accepted:
            self.get_logger().info("GOAL ACCEPTED!")

            # add a callback which runs when a result is received
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback) # call the future callback
        else:
            self.get_logger().warn("GOAL REJECTED")

    def goal_result_callback(self,future):
        status = future.result().status
        result = future.result().result # is the reached number interface made in actions

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("SUCCESS")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("ABORTED")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error("CANCELLED")

        self.get_logger().info(f"Result : {result.message} {result.success}")  #+ str(result.reached_number)
    
    def cancel_goal(self):
        self.get_logger().info("Sending cancel request")
        self.goal_handle_.cancel_goal_async()
         
def main(args=None):
    rclpy.init(args=args)
    node = DumanGoalClient()
    # node.send_goal(arm=True, goal_type=True, target=[0.1, -0.28, 0.36, 0.0, -1.57, 0.0]) # call the send goal function
    node.send_goal(arm=False, goal_type=True, target=[-0.3, -0.28, 0.26, 0.0, 3.17, 0.0]) # call the send goal function

    # node.send_goal(arm=True, goal_type=False, target=node.ready_left) # call the send goal function
    # node.send_goal(arm=False, goal_type=False, target=node.ready_right) # call the send goal function



    node.get_logger().info("TOSHIBAHAHA")
    # time.sleep(2)
    # node.send_goal(arm=False, goal_type=False, target=[-0.6, 0.9, 0.0, 0.0, 0.0, 0.0]) # call the send goal function

    rclpy.spin(node) # then spin the node to wait for result
    rclpy.shutdown()

if __name__=="__main__":
    main()

