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
from pymoveit2.robots import duman_left
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

class DumanHardwareNode(Node):
    def __init__(self):
        super().__init__("duman_hardware")
       
        self.create_service(Dock, "/duman_left/dock", self.dock_control)

        self.create_subscription(Int16, "/duman_left/dist", self.dis_cb, 10)

        self.left_arm_moveit = MoveIt2(
            node=self,
            joint_names=duman_left.joint_names(),
            base_link_name=duman_left.base_link_name(),
            end_effector_name=duman_left.end_effector_name(),
            group_name=duman_left.MOVE_GROUP_ARM,
            callback_group=ReentrantCallbackGroup(),
            follow_joint_trajectory_action_name="duman_left_controller/follow_joint_trajectory",
        )

        self.THRES = 0.05
        self.dist = 0.1

        self.get_logger().info("DOCK SERVER left")
    def dis_cb(self, msg : Int16):
        self.dist = msg.data

    # def dock_control(self, request: Dock.Request, response: Dock.Response):

    #     base_x = request.curr_x
    #     base_y = request.curr_y
    #     base_z = request.curr_z

    #     step = 0.05          # 2 cm
    #     max_radius = 0.2    # 6 cm search
    #     FOUND = False

    #     directions = [(1,0), (0,1), (-1,0), (0,-1)]

    #     x = 0.0
    #     y = 0.0
    #     radius = step

    #     while radius <= max_radius and not FOUND:
    #         for dx, dy in directions:
    #             steps = int(radius / step)

    #             for _ in range(steps):
    #                 x += dx * step
    #                 y += dy * step

    #                 target_pos = [
    #                     base_x + x,
    #                     base_y + y,
    #                     base_z
    #                 ]

    #                 self.get_logger().info(
    #                     f"Spiral move: x={target_pos[0]:.3f}, y={target_pos[1]:.3f}"
    #                 )

    #                 self.left_arm_moveit.set_position_goal(
    #                     position=target_pos,
    #                     frame_id=duman_left.base_link_name(),
    #                     target_link=duman_left.end_effector_name()
    #                 )
    #                 quat = list(quaternion_from_euler(3.14, 0.0, 1.57, "rxyz"))

    #                 self.left_arm_moveit.set_orientation_goal(
    #                     quat_xyzw=quat,
    #                     frame_id=duman_left.base_link_name(),
    #                     target_link=duman_left.end_effector_name()
    #                 )
    #                 waypoint = Pose()

    #                 # position
    #                 waypoint.position.x = target_pos[0]
    #                 waypoint.position.y = target_pos[1]
    #                 waypoint.position.z = target_pos[2]

    #                 # orientation
    #                 qx, qy, qz, qw = quaternion_from_euler(3.14, 0.0, 1.57, axes="rxyz")
    #                 waypoint.orientation.x = qx
    #                 waypoint.orientation.y = qy
    #                 waypoint.orientation.z = qz
    #                 waypoint.orientation.w = qw

    #                 self.left_arm_moveit.execute(self.left_arm_moveit.plan(waypoints=[waypoint],cartesian=True))

    #                 time.sleep(0.05)
    #                 self.get_logger().info("waypoint done")
    #                 if hasattr(self, "dist") and self.dist < self.THRES:
    #                     self.get_logger().info(
    #                         f"Object detected at x={target_pos[0]:.3f}, y={target_pos[1]:.3f}, dist={self.dist:.3f}"
    #                     )

    #                     response.grip_x = target_pos[0]
    #                     response.grip_y = target_pos[1]
    #                     response.success = True
    #                     FOUND = True
    #                     break

    #             if FOUND:
    #                 break

    #         radius += step

    #     if not FOUND:
    #         self.get_logger().warn("Docking failed: object not found")
    #         response.success = False

    #     return response
    
    def dock_control(self, request: Dock.Request, response: Dock.Response):

        base_x = request.curr_x
        base_y = request.curr_y
        base_z = request.curr_z

        step = 0.02        # 5 cm step
        max_dist = 0.1     # ±6 cm search
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

                self.left_arm_moveit.set_position_goal(
                        position=target_pos,
                        frame_id=duman_left.base_link_name(),
                        target_link=duman_left.end_effector_name()
                    )

                self.left_arm_moveit.set_orientation_goal(
                    quat_xyzw=quat,
                    frame_id=duman_left.base_link_name(),
                    target_link=duman_left.end_effector_name()
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
                
                self.left_arm_moveit.execute(self.left_arm_moveit.plan(
                    waypoints=[waypoint],
                    cartesian=True
                ))
                self.left_arm_moveit.wait_until_executed()

                time.sleep(0.05)
                # Check sensor
                if self.dist < self.THRES:
                    self.get_logger().info(
                        f"Object detected at x={target_x:.3f}, dist={self.dist:.3f}"
                    )

                    response.grip_x = target_x
                    response.grip_y = base_y
                    response.success = True
                    FOUND = True
                    break

                offset += step
                self.dist = 0.03
                
            if FOUND:
                break

        if not FOUND:
            self.get_logger().warn("Docking failed: object not found")
            response.success = False

        return response

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
