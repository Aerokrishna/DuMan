#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class CollisionPublisher(Node):
    def __init__(self):
        super().__init__("publish_collision_box")

        self.pub = self.create_publisher(
            CollisionObject,
            "/collision_object",
            10
        )

        self.timer = self.create_timer(1.0, self.publish_object)

    def publish_object(self):
        msg = CollisionObject()
        msg.id = "table"
        msg.operation = CollisionObject.ADD
        msg.header.frame_id = "base"  # <-- IMPORTANT: robot base link

        # --- Define box ---
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [1.0, 0.5, 0.14]  # x, y, z

        # --- Pose ---
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = -0.4
        pose.position.z = 0.07

        # Fill message
        msg.primitives.append(primitive)
        msg.primitive_poses.append(pose)

        # self.get_logger().info("Publishing collision object...")
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CollisionPublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()