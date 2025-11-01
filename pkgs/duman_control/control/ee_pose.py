#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from duman_interfaces.msg import DumanPose
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor

class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')

        # Subscribe to joint states
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.feedback_joint_angle, 10
        )

        self.right_joints = []
        self.left_joints = []

        # TF buffer + listener to read transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Pose publisher
        self.pose_pub = self.create_publisher(DumanPose, "/duman/pose", 10)

        # Timer to publish poses periodically
        self.create_timer(0.1, self.pose_pub_cb)

        self.get_logger().info("JointStateListener (TF-based) started — listening to base→ee_left, base→ee_right")

    def feedback_joint_angle(self, msg: JointState):
        # Just store the latest joint angles (for consistency, even though not used in FK anymore)
        self.left_joints = [msg.position[3], msg.position[5], msg.position[6],
                            msg.position[8], msg.position[10], msg.position[11]]
        self.right_joints = [msg.position[9], msg.position[2], msg.position[7],
                             msg.position[4], msg.position[0], msg.position[1]]

    def pose_pub_cb(self):
        pose_msg = DumanPose()

        try:
            # Lookup left arm transform
            t_left: TransformStamped = self.tf_buffer.lookup_transform(
                "base", "ee_left", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)
            )

            quat_left = [
                t_left.transform.rotation.x,
                t_left.transform.rotation.y,
                t_left.transform.rotation.z,
                t_left.transform.rotation.w
            ]
            eul_left = euler_from_quaternion(quat_left, 'sxyz')

            pose_msg.left_pos_x = t_left.transform.translation.x
            pose_msg.left_pos_y = t_left.transform.translation.y
            pose_msg.left_pos_z = t_left.transform.translation.z
            pose_msg.left_or_x = eul_left[0]
            pose_msg.left_or_y = eul_left[1]
            pose_msg.left_or_z = eul_left[2]

        except Exception as e:
            self.get_logger().warn(f"Left arm TF lookup failed: {e}")
            return

        try:
            # Lookup right arm transform
            t_right: TransformStamped = self.tf_buffer.lookup_transform(
                "base", "ee_right", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)
            )

            quat_right = [
                t_right.transform.rotation.x,
                t_right.transform.rotation.y,
                t_right.transform.rotation.z,
                t_right.transform.rotation.w
            ]
            eul_right = euler_from_quaternion(quat_right, 'sxyz')

            pose_msg.right_pos_x = t_right.transform.translation.x
            pose_msg.right_pos_y = t_right.transform.translation.y
            pose_msg.right_pos_z = t_right.transform.translation.z
            pose_msg.right_or_x = eul_right[0]
            pose_msg.right_or_y = eul_right[1]
            pose_msg.right_or_z = eul_right[2]

        except Exception as e:
            self.get_logger().warn(f"Right arm TF lookup failed: {e}")
            return

        # Publish final pose message
        self.pose_pub.publish(pose_msg)
        # self.get_logger().info("Published DumanPose message from TF transforms")

def main(args=None):
    rclpy.init(args=args)
    node = JointStateListener()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
