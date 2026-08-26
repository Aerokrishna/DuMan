#!/usr/bin/env python3
# Subscribes from /joint_states
# Converts to degrees
# Records one joint's angle vs time and plots it on KeyboardInterrupt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from duman_interfaces.msg import DumanJoints
from geometry_msgs.msg import Pose
import numpy as np
import time
from std_msgs.msg import Int16
from duman_interfaces.srv import Dock
from pymoveit2 import MoveIt2
from pymoveit2.robots import duman_right
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

class DumanHardwareNode(Node):
    def __init__(self):
        super().__init__("duman_hardware")
       
        self.create_service(Dock, "/duman_right/dock", self.dock_control)

        self.create_subscription(Int16, "/duman_right/dist", self.dis_cb, 10)

        self.right_arm_moveit = MoveIt2(
            node=self,
            joint_names=duman_right.joint_names(),
            base_link_name=duman_right.base_link_name(),
            end_effector_name=duman_right.end_effector_name(),
            group_name=duman_right.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup(),
            follow_joint_trajectory_action_name="duman_right_controller/follow_joint_trajectory",
        )
        self.last_time = time.monotonic()

        self.THRES = 12
        self.dist = np.full(10, 100)

        self.get_logger().info("DOCK SERVER RIGHT")

    def dis_cb(self, msg: Int16):
        self.dist[:-1] = self.dist[1:]   # shift left
        self.dist[-1] = msg.data         # insert newest

    def dock_control(self, request: Dock.Request, response: Dock.Response):

        base_x = request.curr_x
        base_y = request.curr_y
        base_z = request.curr_z

        step = 0.02        # 5 cm step
        max_dist = 0.1    # ±6 cm search
        FOUND = False

        quat = quaternion_from_euler(3.14, 0.0, 1.57, axes="rxyz")

        # Search directions: +X first, then -X
        for direction in [+1, -1]:
            offset = 0.0

            while abs(offset) <= max_dist:

                target_x = base_x + direction * offset
                target_pos = [target_x, base_y, base_z]

                self.get_logger().info(
                    f"Linear dock move: x={target_x:.3f}"
                )

                self.right_arm_moveit.set_position_goal(
                        position=target_pos,
                        frame_id=duman_right.base_link_name(),
                        target_link=duman_right.end_effector_name()
                    )

                self.right_arm_moveit.set_orientation_goal(
                    quat_xyzw=quat,
                    frame_id=duman_right.base_link_name(),
                    target_link=duman_right.end_effector_name()
                )
                # Create waypoint
                waypoint = Pose()
                waypoint.position.x = target_x
                waypoint.position.y = base_y
                waypoint.position.z = base_z

                waypoint.orientation.x = quat[0]
                waypoint.orientation.y = quat[1]
                waypoint.orientation.z = quat[2]
                waypoint.orientation.w = quat[3]

                # Cartesian move
                
                self.right_arm_moveit.execute(self.right_arm_moveit.plan(
                    waypoints=[waypoint],
                    cartesian=True
                ))
                # self.right_arm_moveit.wait_until_executed()
                self.delay_(1.0)
                # Check sensor
                self.get_logger().info(f"DISTANCE {self.dist}")
                if not np.all(self.dist > self.THRES):
                    # self.get_logger().info(
                    #     f"Object detected at x={target_x:.3f}, dist={self.dist:.3f}"
                    # )

                    response.grip_x = base_x
                    response.grip_y = base_y
                    response.success = True
                    FOUND = True
                    break
                if self.delay_(1.0):
                    offset += step
                
            if FOUND:
                break

        if not FOUND:
            self.get_logger().warn("Docking failed: object not found")
            response.success = False

        return response

    def delay_(self, period):
        current_time = time.monotonic()
        if current_time - self.last_time >= period:
            self.last_time = current_time
            return True
        return False
    
def main(args=None):
    rclpy.init(args=args)
    node = DumanHardwareNode()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
