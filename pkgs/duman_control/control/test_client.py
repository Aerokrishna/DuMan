#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from duman_interfaces.action import DumanGoal


class DumanGoalClient(Node):
    def __init__(self):
        super().__init__("count_until_server")

        # create action client
        self.duman_goal_client_ = ActionClient(self, DumanGoal, "/duman/goal")


        self.get_logger().info("waiting for server....")
        self.duman_goal_client_.wait_for_server() # you can provide a timer to wait for the server inside
        self.get_logger().info("server found!")

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

        self.duman_goal_client_.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback) 
        
    '''
    Runs async whenever goal request is ACCEPTED/ REJECTED
    ''' 
    def goal_response_callback(self, future):
        # callback to see if goal was accpeted
        self.goal_handle_: ClientGoalHandle = future.result()

        if self.goal_handle_.accepted:
            self.get_logger().info("GOAL ACCEPTED!")

            # add a callback which runs when a result is received
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback) # call the future callback
        else:
            self.get_logger().warn("GOAL REJECTED")

    '''
    Runs async whenever FEEDBACK is received
    '''
    def goal_feedback_callback(self, feedback_msg : DumanGoal):

        # number = feedback_msg.feedback.current_number
        # if number == 4:
        #     self.cancel_goal()
        self.get_logger().info("GOT FEEDBACK ")
    
    '''
    Runs async whenever RESULT is received
    '''
    def goal_result_callback(self,future):
        status = future.result().status
        result = future.result().result # is the reached number interface made in actions

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("SUCCESS")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("ABORTED")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error("CANCELLED")

        self.get_logger().info("Result : ")  #+ str(result.reached_number)
    
    def cancel_goal(self):
        self.get_logger().info("Sending cancel request")
        self.goal_handle_.cancel_goal_async()
         
def main(args=None):
    rclpy.init(args=args)
    node = DumanGoalClient()
    node.send_goal(arm=False, goal_type=False, target=[-0.6, 0.9, 0.0, 0.0, 0.0, 0.0]) # call the send goal function
    node.get_logger().info("TOSHIBAHAHA")
    node.send_goal(arm=True, goal_type=False, target=[0.6, 0.0, 0.0, 0.0, 0.0, 0.0]) # call the send goal function

    rclpy.spin(node) # then spin the node to wait for result
    rclpy.shutdown()

if __name__=="__main__":
    main()