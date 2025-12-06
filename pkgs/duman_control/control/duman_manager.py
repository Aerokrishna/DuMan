#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from duman_interfaces.action import DumanGoal, PickNPlace
import time
from duman_interfaces.srv import GripState, DumanPass
from rclpy.callback_groups import ReentrantCallbackGroup
from google import genai
from prompt import prompt_
from objects import objects_right, objects_left
from essentials import splitTasks, plan_task

class State:
    def __init__(self, name, action_fn, coord=False):
        self.name = name
        self.action = action_fn 
        self.coordinated = coord
        # action_fn is the array of functions which are tasks to be executed in that state
        self.request_sent = False
        self.done = False

class HumanCommandParser(Node):
    def __init__(self, task_manager):
        super().__init__("dual_arm_planner_node")
        self.task_manager = task_manager

        self.get_logger().info("Dual Arm Planner Node started.")
        self.get_logger().info("Waiting for user commands...")

        # Timer that checks for user input
        self.timer = self.create_timer(0.1, self.check_user_input)

        self.input_buffer = ""

        self.llm_output = None

    def check_user_input(self):
        try:
            import sys, select
            if select.select([sys.stdin], [], [], 0.0)[0]:
                command = sys.stdin.readline().strip()
                if command:
                    self.process_command(command)
        except Exception as e:
            self.get_logger().error(f"Input error: {e}")

    def process_command(self, command: str):
        self.get_logger().info(f"Received command: {command}")

        right_objs = list(objects_right.keys())
        left_objs = list(objects_left.keys())   # FIX: was wrong earlier

        try:
            plan = plan_task(command, right_objs, left_objs)
        except Exception as e:
            self.get_logger().error(f"LLM Error: {e}")
            return

        left_seq, right_seq = splitTasks(plan)
        self.get_logger().info(f"RIGHT ARM : {right_seq}")
        self.get_logger().info(f"LEFT ARM : {left_seq}")

        # Send new plan to FSM
        self.task_manager.receive_new_plan(left_seq, right_seq)

class TaskManager(Node):
    def __init__(self):
        super().__init__('fsm_node')

        self.current_right = None
        self.current_left = None

        self.left_states = []
        self.right_states = []

        self.right_index = 0
        self.left_index = 0

        # self.get_logger().info(f"[FSM] Starting at state: {self.current.name}")
        self.timer = self.create_timer(0.5, self.step)

        # action server clients for left and right arms
        self.pnp_left_client_ = ActionClient(self, PickNPlace, "/duman/pnp_left")
        self.pnp_right_client_ = ActionClient(self, PickNPlace, "/duman/pnp_right")
        self.duman_pass_client = self.create_client(DumanPass, "/duman/pass")

        self.llm_response = None

        self.get_logger().info("waiting for server....")
        self.pnp_left_client_.wait_for_server() # you can provide a timer to wait for the server inside
        self.pnp_right_client_.wait_for_server() # you can provide a timer to wait for the server inside
        self.get_logger().info("server found!")

    def step(self):

        if self.current_left is not None and self.current_right is not None:
            if self.current_left.coordinated and self.current_right.coordinated:

                if not self.current_left.request_sent and not self.current_right.request_sent:
                    self.get_logger().info("BOTH ARMS READY → SENDING PASS")
                    self.current_left.action()   # or right, both call same service
                    self.current_left.request_sent = True
                    self.current_right.request_sent = True

        # SEND ONLY RIGHT ARM COMMANDS AND SWITCH STATES IN THE SEQ
        if self.current_right is not None:
            if not self.current_right.coordinated:
                if not self.current_right.request_sent:
                    self.get_logger().info('RIGHT ARM PICK IDEALL')
                    self.current_right.action()
                    self.current_right.request_sent = True

        # if result is received
            if self.current_right.done and self.current_right.request_sent:
                if self.right_index + 1 >= len(self.right_states):
                    self.get_logger().info("RIGHT ARM TASKS DONE")

                self.right_index += 1
                self.current_right = self.right_states[self.right_index]
                self.get_logger().info(f"RIGHT ARM Transition → {self.current_right.name}")

        # SEND ONLY LEFT ARM COMMANDS AND SWITCH STATES IN THE SEQ
        if self.current_left is not None:
            if not self.current_left.coordinated:
                if not self.current_left.request_sent:
                    self.current_left.action()
                    self.current_left.request_sent = True

            if self.current_left.done and self.current_left.request_sent:
                if self.left_index + 1 >= len(self.left_states):
                    self.get_logger().info("LEFT ARM TASKS DONE")

                self.left_index += 1
                self.current_left = self.left_states[self.left_index]
                self.get_logger().info(f"LEFT ARM Transition → {self.current_left.name}")

    def send_pnp_cmd(self, pick, arm, obj):

        # Define your goal as your custom action
        goal = PickNPlace.Goal()
        goal.pick = pick # true if picking
        goal.object_id = obj

        if arm: # left arm 
            self.pnp_left_client_.send_goal_async(goal).add_done_callback(self.goal_response_callback) 
        else:
            self.pnp_right_client_.send_goal_async(goal).add_done_callback(self.goal_response_callback) 

    def send_pass_cmd(self, to_arm):
        req = DumanPass.Request()

        req.to_arm = to_arm
        future = self.duman_pass_client.call_async(req)
        future.add_done_callback(self.pass_result_callback)

    def pass_result_callback(self, future):
        try:
            self.current_right.done = True
            self.current_left.done = True
            
            # self.get_logger().info(f'Service response: {self.response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def goal_response_callback(self, future):
        # callback to see if goal was accpeted
        self.goal_handle_: ClientGoalHandle = future.result()

        if self.goal_handle_.accepted:
            self.get_logger().info("GOAL ACCEPTED!")
            self.goal_handle_.get_result_async().add_done_callback(self.pnp_result_callback) # call the future callback
        else:
            self.get_logger().warn("GOAL REJECTED")

    def pnp_result_callback(self,future):
        status = future.result().status
        result = future.result().result # is the reached number interface made in actions

        if status == GoalStatus.STATUS_SUCCEEDED:
            if result.message == "right":
                self.current_right.done = True
            else:
                self.current_left.done = True
            self.get_logger().info("SUCCESS")

        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("ABORTED")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error("CANCELLED")

        self.get_logger().info(f"Result : {result.message} {result.success}")  #+ str(result.reached_number)
    
    def cancel_goal(self):
        self.get_logger().info("Sending cancel request")
        self.goal_handle_.cancel_goal_async()

    def generate_plan(self, left_seq, right_seq):

        for step in left_seq:
            if step[1] == "pick":
                self.left_states.append(State("left_pick", 
                                            lambda obj_id=step[2]: self.send_pnp_cmd(pick=True, arm=True, obj=obj_id)))
            elif step[1] == "place":
                self.left_states.append(State("left_place", 
                                            lambda obj_id=step[2]: self.send_pnp_cmd(pick=False, arm=True, obj=obj_id)))

            elif step[1] == "pass":
                if step[0] == "left":
                    self.left_states.append(State("left_pass", 
                                                lambda : self.send_pass_cmd(to_arm=False), coord=True))
                else:
                    self.left_states.append(State("left_pass", 
                                                lambda : self.send_pass_cmd(to_arm=True), coord=True))

        for step in right_seq:
            if step[1] == "pick":
                self.right_states.append(State("right_pick", 
                                            lambda obj_id=step[2]: self.send_pnp_cmd(pick=True, arm=False, obj=obj_id)))
            elif step[1] == "place":
                self.right_states.append(State("right_place", 
                                            lambda obj_id=step[2]: self.send_pnp_cmd(pick=False, arm=False, obj=obj_id)))

            elif step[1] == "pass":
                if step[0] == "right":
                    self.right_states.append(State("right_pass", 
                                                lambda : self.send_pass_cmd(to_arm=True), coord=True))
                else:
                    self.right_states.append(State("right_pass", 
                                                lambda : self.send_pass_cmd(to_arm=False), coord=True))

    def receive_new_plan(self, left_seq, right_seq):
        # Reset
        self.left_states = []
        self.right_states = []
        self.left_index = 0
        self.right_index = 0

        # need to cancel the current request for both right and left arms

        # Generate new states
        self.generate_plan(left_seq, right_seq)

        # Set current states
        self.current_left = self.left_states[0] if self.left_states else None
        self.current_right = self.right_states[0] if self.right_states else None

        self.get_logger().info("New plan loaded into FSM.")

def main(args=None):
    rclpy.init(args=args)

    task_manager = TaskManager()
    parser = HumanCommandParser(task_manager)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(task_manager)
    executor.add_node(parser)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        task_manager.destroy_node()
        parser.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()