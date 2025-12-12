#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse
from duman_interfaces.action import PickNPlace, DumanGoal
from duman_interfaces.srv import GripState
from duman_interfaces.msg import Object
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
from rclpy.action.client import ClientGoalHandle, GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
import time
from objects import objects_right
import copy

# it performs sequential pick and place

# it is responsible for cancelling the request if necessary
# this server reads from the camera and object poses, requests replan
# gripping retry tree
# retry if planning fails multiple times

class PicknPlace(Node):
    def __init__(self):
        super().__init__("pnp_right")
        self.goal_handle_ : ServerGoalHandle = None

        self.goal_lock_ = threading.Lock()
        # create action client
        self.duman_right_goal_client_ = ActionClient(self, DumanGoal, "/duman/goal_right", callback_group=ReentrantCallbackGroup())

        self.joint_goal_server_ = ActionServer(
            self, 
            PickNPlace,  
            "/duman/pnp_right",
            goal_callback=self.goal_callback, 
            cancel_callback=self.cancel_callback, 
            execute_callback=self.pnp_callback, 
            callback_group=ReentrantCallbackGroup()) 
        
        self.duman_grip_client = self.create_client(GripState, "/duman/grip_state", callback_group=ReentrantCallbackGroup())

        self.create_subscription(Object, "/duman/objects", self.object_cb, 10)

        self.arm_done = False
        self.state = 0
        self.goal_sent = False
        self.ik_failed = False

        self.object_poses = objects_right
        self.object_height = 0.05 # 5cm
        self.approach_ht = 0.1

        self.get_logger().info("duman right pick and place server")

        self.last_time = time.monotonic()
        self.cnt = 0

        self.get_logger().info("waiting for server....")
        self.duman_right_goal_client_.wait_for_server() # you can provide a timer to wait for the server inside
        self.get_logger().info("server found!")
    
    def object_cb(self, msg : Object):
        for obj, pose in zip(msg.obj_right, msg.obj_pose_right):
            if obj != "bowl":
                self.object_poses[obj] = [pose.x, pose.y, 0.18, 3.14, 0.0, 1.54] 

    def goal_callback(self, goal_request: PickNPlace.Goal):
        
        # reject the goal if a goal is already executing
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().error("GOAL ACTIVE...rejecting new goal")
                return GoalResponse.REJECT

        if goal_request.object_id not in self.object_poses:
            self.get_logger().error("Object not found")
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle): # goal handle is going to be cancelled as client will cancel a particular goal
        self.get_logger().info("cancel request received...")
        return CancelResponse.ACCEPT
    
    def cancel_n_reset(self):
        result = PickNPlace.Result()
        result.success = False
        self.state = 0
        self.goal_sent = False
        self.goal_handle_ = None
        self.ik_failed = False

        result.message = "canceled"

        return result
    
    def pnp_callback(self, goal_handle : ServerGoalHandle):
        self.state = 1
        # self.get_logger().info("PASSING REQUEST RECEIVED")
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        obj_pose = copy.deepcopy(self.object_poses[goal_handle.request.object_id])
        align_pose = copy.deepcopy(self.object_poses[goal_handle.request.object_id])
        align_pose[2] += (self.object_height + self.approach_ht)

        obj_pose[2] = 0.165
        align_pose[0] += 0.03
        obj_pose[0] += 0.03

        # align_pose[2] += (self.object_height)
        # align_pose[1] += (self.object_height)

        # align_pose[1] = -0.25

        while True:
            if goal_handle.is_cancel_requested or self.ik_failed:
                self.get_logger().warn("CANCEL RECEIVED — Stopping arm safely")
                goal_handle.abort()
                result = self.cancel_n_reset()
                return result 

            time.sleep(0.1)
            # if self.delay_(0.1):
            if self.state == 1:
                # self.get_logger().info(f"Right Arm Aligning to object ")

                if not self.goal_sent:
                    self.arm_done = False
                    
                    self.send_goal(arm=False, goal_type=True, target=align_pose)
                    self.goal_sent = True
            
            elif self.state == 2:
                # self.get_logger().info(f"Moving towards object")

                if not self.goal_sent:
                    self.arm_done = False
                    """
                    OBJECT POSE
                    """
                    self.send_goal(arm=False, goal_type=True, target=obj_pose)
                    self.goal_sent = True

            elif self.state == 3 and self.delay_(1.0):

                if not self.goal_sent:
                    # self.get_logger().info(f"Grip Command")

                    # if pick is true grip state is true means close the gripper else false then open
                    self.send_grip_cmd(arm=False, grip_state=goal_handle.request.pick)
                    self.goal_sent = True

            elif self.state == 4 and self.delay_(1.0):
                # self.get_logger().info(f"Right Arm Aligning to object ")

                if not self.goal_sent:
                    self.arm_done = False

                    self.send_goal(arm=False, goal_type=True, target=align_pose)
                    self.goal_sent = True
                    
            if self.state == 5:
                # self.get_logger().info(f"IDLING")
                self.state = 0
                self.goal_sent = False
                self.goal_handle_ = None

                break
        
        result = PickNPlace.Result()
        result.success = True
        result.message = "right"

        if goal_handle.is_active:
            goal_handle.succeed()
        else:
            self.get_logger().warn("Goal already terminated before success call")
            return result

        self.get_logger().info("GOAL FINISH RETURNING SUCCESS!")
        return result
    
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
            self.ik_failed = True
            self.get_logger().warn("GOAL REJECTED")

    # a callback to signify completion of an arm motion task
    # the motion result will include which arm has completed the motion task
    def motion_result_callback(self,future):
        status = future.result().status
        result = future.result().result # is the reached number interface made in actions

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.state+=1
            self.goal_sent = False
            
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("ABORTED")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error("CANCELLED")

        # self.get_logger().info(f"Result : {result.message} {result.success}")  #+ str(result.reached_number)
    
    def cancel_goal(self):
        self.get_logger().info("Sending cancel request")
        self.goal_handle_.cancel_goal_async()

    def delay_(self, period):
        current_time = time.monotonic()
        if current_time - self.last_time >= period:
            self.last_time = current_time
            return True
        return False
    
def main(args=None):
    rclpy.init(args=args)
    node = PicknPlace()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__=="__main__":
    main()

