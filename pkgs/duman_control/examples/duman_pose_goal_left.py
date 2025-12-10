#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import duman_left
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

def main():
    rclpy.init()
    # Create node for this example
    node = Node("pose_goal")

    # Declare parameters for position and orientation
    node.declare_parameter("position", [0.005, -0.2, 0.28])
    q = quaternion_from_euler(1.57, -1.57, 0.0)
    quat = list(q)  # Quaternion in xyzw format
    # quat = [0.703, -0.588, 0.398, -0.004]
    node.declare_parameter("quat_xyzw", quat)
    node.declare_parameter("cartesian", False)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=duman_left.joint_names(),
        base_link_name=duman_left.base_link_name(),
        end_effector_name=duman_left.end_effector_name(),
        group_name=duman_left.MOVE_GROUP_ARM,
        callback_group=callback_group,
        follow_joint_trajectory_action_name="duman_left_controller/follow_joint_trajectory",
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = node.get_parameter("cartesian").get_parameter_value().bool_value

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.set_position_goal(position=position, frame_id=duman_left.base_link_name(), target_link=duman_left.end_effector_name())
    moveit2.set_orientation_goal(quat_xyzw=quat_xyzw, frame_id=duman_left.base_link_name(), target_link=duman_left.end_effector_name())

    print(moveit2.compute_ik(position=position, quat_xyzw=quat))
    # moveit2.compute_fk(joint_state=[1.0,1.0,2.0,1.0,0.0], fk_link_names=duman.joint_names())
    # moveit2._plan_cartesian_path(frame_id="base")
    plan = moveit2.plan()
    
    # print(plan)
    moveit2.execute(plan)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()
