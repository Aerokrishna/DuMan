#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from duman_interfaces.action import DumanGoal
import time
from duman_interfaces.srv import GripState

'''
each state has its own function
and a timer object is created with a new timer callback function

passing operation
right arm goes to a pick up position 
left arm comes to the pass position

right arm closes the gripper after 5 seconds
left arm opens the gripper

right arm comes to a position parallel to the pass position
right arm moves closer to the pass position

left arm closes the gripper
right arm opens the gripper

left arm moves to a drop posion
left arm opens the gripper
'''

'''
a state machine architecture

with different states
with respect to a particular state it will be spinning a function
and when that function is over we just move to the next state = we know when the function is over based on a condition
so a state + function + complete_condition
delay function for time based state changes
a blackboard which just contains flags which are referred to change the states

'''

class DumanGoalClient(Node):
    def __init__(self):
        super().__init__("count_until_server")

        # create action client
        self.duman_left_goal_client_ = ActionClient(self, DumanGoal, "/duman/goal_left")
        self.duman_right_goal_client_ = ActionClient(self, DumanGoal, "/duman/goal_right")

        self.duman_right_grip_client = self.create_client(GripState, "/duman/grip_state_right")

        self.get_logger().info("waiting for server....")
        self.duman_right_goal_client_.wait_for_server() # you can provide a timer to wait for the server inside
        # self.duman_left_goal_client_.wait_for_server() # you can provide a timer to wait for the server inside
        self.get_logger().info("server found!")

        self.ready_right = [-0.3, 0.5, -1.57, 0.0, 0.0, 1.57]
        self.ready_left = [0.3, -0.5, 1.57, 0.5, 0.7, 1.57]

        self.zero_right = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.zero_left = [0.0, 0.0, 0.0, 0.8, -0.8, 0.8]

        self.pick = [-0.7, 0.35, -0.63, 1.57, -1.0, 1.57]
        self.pass_ = [-0.5, 0.5, -2.0, 0.0, -0.60, 0.0]

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

    def send_grip_cmd(self, arm, grip_state):
        # Create a request for the ArucoSW service, to get the pick and drop coordinates.
        self.get_logger().info("REQESTING GRIPPER CONTROL!")

        req = GripState.Request()
        req.grip_state = grip_state
        req.arm = arm
        # Call the service asynchronously
        future = self.duman_right_grip_client.call_async(req)
        future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        try:
            self.response = future.result()
            # self.get_logger().info(f'Service response: {self.response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

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

    node.send_goal(arm=False, goal_type=False, target=node.ready_right) # call the send goal function
    time.sleep(4)
    node.send_goal(arm=False, goal_type=False, target=node.pick) # call the send goal function
    time.sleep(4)
    node.send_grip_cmd(arm=False, grip_state=True) # close gripper
    time.sleep(2)
    node.send_goal(arm=False, goal_type=False, target=node.pass_) # call the send goal function
    time.sleep(4)
    node.send_goal(arm=False, goal_type=False, target=node.ready_right) # call the send goal function
    time.sleep(4)
    node.send_goal(arm=False, goal_type=False, target=node.zero_right) # call the send goal function

    node.get_logger().warn("MISSION COMPLETE")

    rclpy.spin(node) # then spin the node to wait for result
    rclpy.shutdown()

if __name__=="__main__":
    main()












    # node.send_goal(arm=True, goal_type=True, target=[0.1, -0.28, 0.36, 0.0, -1.57, 0.0]) # call the send goal function
    # node.send_goal(arm=False, goal_type=True, target=[-0.3, -0.28, 0.26, 0.0, 3.17, 0.0]) # call the send goal function

    # arm-True = LEFT # arm-False = RIGHT # goal_type-True = POSE GOAL goal_type-False = JOINT GOAL