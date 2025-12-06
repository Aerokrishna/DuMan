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
        self.request_sent = False
        self.done = False

class HumanCommandParser(Node):
    def __init__(self, task_manager):
        super().__init__("dual_arm_planner_node")
        self.task_manager = task_manager

        self.get_logger().info("Dual Arm Planner Node started.")
        self.get_logger().info("Waiting for user commands...")

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
        left_objs = list(objects_left.keys())

        try:
            plan = plan_task(command, right_objs, left_objs, self.task_manager.right_holding, self.task_manager.left_holding)
            self.get_logger().info(f'RIGHT HOLDS : {self.task_manager.right_holding}')
            self.get_logger().info(f'LEFT HOLDS : {self.task_manager.left_holding}')

        except Exception as e:
            self.get_logger().error(f"LLM Error: {e}")
            return

        left_seq, right_seq = splitTasks(plan)
        self.get_logger().info(f"RIGHT ARM : {right_seq}")
        self.get_logger().info(f"LEFT ARM : {left_seq}")

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

        self.timer = self.create_timer(0.5, self.step)

        self.pnp_left_client_ = ActionClient(self, PickNPlace, "/duman/pnp_left")
        self.pnp_right_client_ = ActionClient(self, PickNPlace, "/duman/pnp_right")
        self.duman_pass_client = self.create_client(DumanPass, "/duman/pass")

        # ---------------------------
        # NEW: separate goal handles
        # ---------------------------
        self.goal_handle_left = None
        self.goal_handle_right = None

        self.llm_response = None
        self.pass_in_progress = False
        self.pending_new_plan = None

        self.right_holding = ''
        self.left_holding = ''


        self.get_logger().info("waiting for server....")
        self.pnp_left_client_.wait_for_server()
        self.pnp_right_client_.wait_for_server()
        self.get_logger().info("server found!")

    def step(self):

        if self.current_left is not None and self.current_right is not None:
            if self.current_left.coordinated and self.current_right.coordinated:

                if not self.current_left.request_sent and not self.current_right.request_sent:
                    self.get_logger().info("BOTH ARMS READY → SENDING PASS")
                    self.current_left.action()
                    self.current_left.request_sent = True
                    self.current_right.request_sent = True
                    self.pass_in_progress = True

        if self.current_right is not None:
            if not self.current_right.coordinated:
                if not self.current_right.request_sent:
                    self.get_logger().info('RIGHT ARM PICK IDEALL')
                    self.current_right.action()
                    self.current_right.request_sent = True

            if self.current_right.done and self.current_right.request_sent:
                if self.right_index + 1 >= len(self.right_states):
                    self.get_logger().info("RIGHT ARM TASKS DONE")

                else:
                    self.right_index += 1
                    self.current_right = self.right_states[self.right_index]
                    self.get_logger().info(f"RIGHT ARM Transition → {self.current_right.name}")

        if self.current_left is not None:
            if not self.current_left.coordinated:
                if not self.current_left.request_sent:
                    self.current_left.action()
                    self.current_left.request_sent = True

            if self.current_left.done and self.current_left.request_sent:
                if self.left_index + 1 >= len(self.left_states):
                    self.get_logger().info("LEFT ARM TASKS DONE")

                else:
                    self.left_index += 1
                    self.current_left = self.left_states[self.left_index]
                    self.get_logger().info(f"LEFT ARM Transition → {self.current_left.name}")

    def send_pnp_cmd(self, pick, arm, obj):

        goal = PickNPlace.Goal()
        goal.pick = pick
        goal.object_id = obj

        if arm: 
            self.pnp_left_client_.send_goal_async(goal).add_done_callback(
                lambda f: self.goal_response_callback(f, arm=True))
        else:
            self.pnp_right_client_.send_goal_async(goal).add_done_callback(
                lambda f: self.goal_response_callback(f, arm=False))

    def send_pass_cmd(self, to_arm):
        req = DumanPass.Request()

        req.to_arm = to_arm

        if req.to_arm:
                # right → left
                self.left_holding = self.right_holding
                self.right_holding = ""
                self.get_logger().info(f"[PASS] RIGHT → LEFT : {self.left_holding}")

        else:
            # left → right
            self.right_holding = self.left_holding
            self.left_holding = ""
            self.get_logger().info(f"[PASS] LEFT → RIGHT : {self.right_holding}")

        future = self.duman_pass_client.call_async(req)
        future.add_done_callback(self.pass_result_callback)

    def pass_result_callback(self, future):
        try:
            result = future.result()

            # to_arm=True  --> pass TO left_arm
            
            # both states complete
            self.current_left.done = True
            self.current_right.done = True

            # handle deferred plans
            if self.pending_new_plan:
                left_seq, right_seq = self.pending_new_plan
                self.pending_new_plan = None
                self.pass_in_progress = False
                self.receive_new_plan(left_seq, right_seq)

        except Exception as e:
            self.get_logger().error(f"PASS ERROR: {e}")


    def goal_response_callback(self, future, arm):
        goal_handle = future.result()

        if arm:
            self.goal_handle_left = goal_handle
        else:
            self.goal_handle_right = goal_handle

        if goal_handle.accepted:
            self.get_logger().info("GOAL ACCEPTED!")
            goal_handle.get_result_async().add_done_callback(self.pnp_result_callback)
        else:
            self.get_logger().warn("GOAL REJECTED")

    def pnp_result_callback(self, future):
        result_msg = future.result()
        status = result_msg.status
        result = result_msg.result

        if status == GoalStatus.STATUS_SUCCEEDED:
            arm = result.message  # "right" or "left"

            if "pick" in (self.current_right.name if arm=="right" else self.current_left.name):
                # PICK SUCCESS
                if arm == "right":
                    self.right_holding = self.current_right.action.__closure__[0].cell_contents
                else:
                    self.left_holding = self.current_left.action.__closure__[0].cell_contents

            elif "place" in (self.current_right.name if arm=="right" else self.current_left.name):
                # PLACE SUCCESS
                if arm == "right":
                    self.right_holding = ""
                else:
                    self.left_holding = ""

            # mark done
            if arm == "right":
                self.current_right.done = True
            else:
                self.current_left.done = True

            self.get_logger().info(f"[PNP] {arm.upper()} DONE  holding L={self.left_holding}  R={self.right_holding}")

        else:
            self.get_logger().error("PNP FAILED")


    # -------------------------------------------------
    # FIXED CANCEL → cancel per arm
    # -------------------------------------------------
    def cancel_goal(self, arm=None):
        self.get_logger().info("Sending cancel request")

        if arm == "left" and self.goal_handle_left:
            self.goal_handle_left.cancel_goal_async()

        elif arm == "right" and self.goal_handle_right:
            self.goal_handle_right.cancel_goal_async()

        else:  
            # cancel both
            if self.goal_handle_left:
                self.goal_handle_left.cancel_goal_async()
            if self.goal_handle_right:
                self.goal_handle_right.cancel_goal_async()

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

        if self.pass_in_progress:
            self.get_logger().warn("PASS IN PROGRESS → Deferring new plan.")
            self.pending_new_plan = (left_seq, right_seq)
            return

        self.get_logger().info("Resetting FSM for new plan...")

        try:
            if self.goal_handle_left:
                self.goal_handle_left.cancel_goal_async()
            if self.goal_handle_right:
                self.goal_handle_right.cancel_goal_async()
        except:
            pass

        self.left_states = []
        self.right_states = []
        self.left_index = 0
        self.right_index = 0

        self.generate_plan(left_seq, right_seq)
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
