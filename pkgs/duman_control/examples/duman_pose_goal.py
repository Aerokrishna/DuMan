#!/usr/bin/env python3
"""
Example of moving to a pose goal following a small circular path.
"""

from threading import Thread
import math
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import duman_right
from geometry_msgs.msg import Pose


def main():
    rclpy.init()
    node = Node("pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.0, -0.23, 0.36])
    quat = [0.703, -0.588, 0.398, -0.004]
    node.declare_parameter("quat_xyzw", quat)
    node.declare_parameter("cartesian", True)

    callback_group = ReentrantCallbackGroup()
    moveit2 = MoveIt2(
        node=node,
        joint_names=duman_right.joint_names(),
        base_link_name=duman_right.base_link_name(),
        end_effector_name=duman_right.end_effector_name(),
        group_name=duman_right.MOVE_GROUP_ARM,
        callback_group=callback_group,
        follow_joint_trajectory_action_name="duman_right_controller/follow_joint_trajectory",
    )

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value


    # 2. Optionally set the current pose as the target (some versions require it)
    moveit2.set_pose_goal(position=position, quat_xyzw=quat_xyzw)

    # Define base pose
    target_pose = Pose()
    target_pose.position.x = 0.0
    target_pose.position.y = -0.33
    target_pose.position.z = 0.36
    target_pose.orientation.x = quat_xyzw[0]
    target_pose.orientation.y = quat_xyzw[1]
    target_pose.orientation.z = quat_xyzw[2]
    target_pose.orientation.w = quat_xyzw[3]

    # ---- Generate circular waypoints ----
    radius = 0.08  # 5 cm circle
    num_points = 8
    waypoints = []

    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        p = Pose()
        p.position.x = target_pose.position.x
        p.position.y = target_pose.position.y + radius * math.cos(angle)
        p.position.z = target_pose.position.z + radius * math.sin(angle)
        p.orientation = target_pose.orientation
        waypoints.append(p)

    # Close the circle
    waypoints.append(waypoints[0])

    node.get_logger().info(f"Planning Cartesian circular path with {len(waypoints)} waypoints...")

    plan = moveit2.plan(cartesian=True, waypoints=waypoints)
    moveit2.execute(plan)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
