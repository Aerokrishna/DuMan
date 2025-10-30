#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from duman_interfaces.action import DumanGoal
import time
from duman_interfaces.srv import GripState, DumanPass
from rclpy.callback_groups import ReentrantCallbackGroup
from google import genai
from prompt import prompt_

'''
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


class State:
    """Represents a single FSM state."""
    def __init__(self, name, action_fn):
        self.name = name
        self.actions = action_fn 
        # action_fn is the array of functions which are tasks to be executed in that state
        self.num_actions = len(self.actions)
        self.request_sent = 0
        self.done = 0

class FSMNode(Node):
    def __init__(self):
        super().__init__('fsm_node')

        self.states = []

        # Define states in sequential order
        # self.states = [
        #     State("LEFT N RIGHT POSE", [lambda: self.send_goal(arm=True, goal_type=True, target=ready_left_pose),
        #                         lambda: self.send_goal(arm=False, goal_type=True, target=ready_right_pose)]),
        #     State("RIGHT", [lambda: self.send_goal(arm=False, goal_type=False, target=self.ready_right)]),
        #     State("LEFT", [lambda: self.send_goal(arm=True, goal_type=False, target=self.zero_left)]),
        # ]

        # self.states = [
        #     State("LEFT", [lambda: self.send_pass_cmd(to_arm=True)]),
        # ]
        self.states = []
        self.index = 0
        self.current = None

        # self.get_logger().info(f"[FSM] Starting at state: {self.current.name}")
        self.timer = self.create_timer(0.5, self.step)

        # action server clients for left and right arms
        self.duman_left_goal_client_ = ActionClient(self, DumanGoal, "/duman/goal_left")
        self.duman_right_goal_client_ = ActionClient(self, DumanGoal, "/duman/goal_right")

        self.duman_grip_client = self.create_client(GripState, "/duman/grip_state")

        self.duman_pass_client = self.create_client(DumanPass, "/duman/pass")

        self.llm_response = None

        self.get_logger().info("waiting for server....")
        self.duman_right_goal_client_.wait_for_server() # you can provide a timer to wait for the server inside
        self.duman_left_goal_client_.wait_for_server() # you can provide a timer to wait for the server inside
        self.get_logger().info("server found!")

    def step(self):
        # send the request (call the send client funciton here)
        if not self.current.request_sent:
            for task in self.current.actions:
                self.current.request_sent += 1

                task()

        self.get_logger().info(f"{self.current.name}")
        # if result is received
        if self.current.done + self.current.request_sent == 2 * self.current.num_actions:
            if self.index + 1 >= len(self.states):
                self.get_logger().info("[FSM] Final state reached. Stopping node.")
                self.timer.cancel()
                return

            self.index += 1
            self.current = self.states[self.index]
            self.previous = self.states[self.index-1]
            self.get_logger().info(f"[FSM] Transition → {self.current.name}")
    
    def send_goal(self, arm:bool, goal_type, target):
        print("AAAAAAAA : ", arm)
        # Define your goal as your custom action
        goal = DumanGoal.Goal()

        goal.arm = arm 

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

        if arm == True:
            self.get_logger().info("POSE Goal sending LEFT")
            self.duman_left_goal_client_.send_goal_async(goal).add_done_callback(self.goal_response_callback) 
        
        else:
            self.get_logger().info("POSE Goal sending RIGHT")
            self.duman_right_goal_client_.send_goal_async(goal).add_done_callback(self.goal_response_callback) 

    def send_pass_cmd(self, to_arm):
        req = DumanPass.Request()

        req.to_arm = to_arm
        future = self.duman_pass_client.call_async(req)
        future.add_done_callback(self.result_callback)

    def send_grip_cmd(self, arm, grip_state):
        # Create a request for the ArucoSW service, to get the pick and drop coordinates.

        req = GripState.Request()
        req.grip_state = grip_state
        req.arm = arm

        if not arm:
            self.get_logger().info("REQESTING GRIPPER CONTROL RIGHT!")

            # Call the service asynchronously
            future = self.duman_grip_client.call_async(req)
            future.add_done_callback(self.result_callback)
        
        else :
            # Call the service asynchronously
            self.get_logger().info("REQESTING GRIPPER CONTROL LEFT!")

            future = self.duman_grip_client.call_async(req)
            future.add_done_callback(self.result_callback)
    
    
    def result_callback(self, future):
        try:
            if future.result().success:
                self.current.done += 1
            
            # self.get_logger().info(f'Service response: {self.response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def goal_response_callback(self, future):
        # callback to see if goal was accpeted
        self.goal_handle_: ClientGoalHandle = future.result()

        if self.goal_handle_.accepted:
            self.get_logger().info("GOAL ACCEPTED!")
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
            self.current.done += 1
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
    node = FSMNode()

    try:
        user_command = input("Hi I am Duman how can I help you?  ")
        client = genai.Client(api_key="AIzaSyB8qsmtPG5W5-zIkiHuPFl7AZSNd9UTusM")

        # from the camera
        left_side_objects = ["basket", "banana"]
        right_side_objects = ["apple", "orange", "cup"]

        response = client.models.generate_content(
            model="gemini-2.5-pro",
            contents=[prompt_(left_side_objects, right_side_objects, user_command)],
        )

        # try:
        node.llm_response = eval(response.text)
        print("\n--- Parsed Plan ---")

        # except Exception as e:
        #     print("\nCould not parse output, raw text:")
        #     print(response.text)
        ref_fun = []

        for actions in node.llm_response:
            functions = []
            for action in actions:
                intent = action[0]

                if action[1] == 'left':
                    node.get_logger().info("LEFT ARM")
                    arm = True
                elif action[1] == 'right':
                    node.get_logger().info("RIGHT ARM")
                    arm = False

                if intent == "move":
                    if arm :
                        functions.append(lambda : node.send_goal(arm=True, goal_type=True, target=[-0.1, -0.2, 0.25, 1.57, 0.0, 0.0]))
                    else:
                        functions.append(lambda : node.send_goal(arm=False, goal_type=True, target=[-0.1, -0.2, 0.25, 1.57, 0.0, 0.0]))   
                    state_name = "MOVE"+str(arm)
                    ref_fun.append(state_name)

                elif intent == "transfer":

                    state_name = "TRANSFER"
                    ref_fun.append(state_name)

                    if arm:
                        functions.append(lambda : node.send_pass_cmd(to_arm=True))
                    else:
                        functions.append(lambda : node.send_pass_cmd(to_arm=False))

                elif intent == "grip":
                    state_name = "GRIP"
                    ref_fun.append(state_name)

                    if arm :    
                        functions.append(lambda : node.send_grip_cmd(grip_state=True, arm=True))
                    else:
                        functions.append(lambda : node.send_grip_cmd(grip_state=True, arm=False))

                elif intent == "ungrip":
                    state_name = "UNGRIP"
                    ref_fun.append(state_name)

                    if arm :    
                        functions.append(lambda : node.send_grip_cmd(grip_state=False, arm=True))
                    else:
                        functions.append(lambda : node.send_grip_cmd(grip_state=False, arm=False))

            node.states.append(State(state_name, functions))

        node.current = node.states[0]

        node.get_logger().info(f'{node.llm_response}')
        node.get_logger().info(f'{ref_fun}')

        if node.llm_response is not None:
            rclpy.spin(node)

        else:
            node.get_logger().info("LLM MESSED AAAA")

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down FSM node.")
    finally:
        node.timer.cancel()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
