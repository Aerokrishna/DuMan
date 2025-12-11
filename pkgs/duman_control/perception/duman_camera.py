#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from ultralytics import YOLO

# TF2
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from duman_interfaces.msg import Point, Object
from tf2_ros import Buffer, TransformListener

class RGBDViewer(Node):
    def __init__(self):
        super().__init__('rgbd_viewer')

        self.bridge = CvBridge()

        # Subscribers with sync
        rgb_sub = message_filters.Subscriber(self, Image, '/ascamera/camera_publisher/rgb0/image')
        depth_sub = message_filters.Subscriber(self, Image, '/ascamera/camera_publisher/depth0/image_raw')
        
        # Synchronizer
        ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub],
            queue_size=10,
            slop=0.1
        )
        ts.registerCallback(self.callback)

        self.obj_pub = self.create_publisher(Object, "/duman/objects", 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.rgb_img = None
        self.depth_img = None
        self.visual = True

        # YOLO
        self.yolo_model = YOLO("yolov8n.pt")
        self.names = (
            self.yolo_model.model.names
            if hasattr(self.yolo_model, "model") and hasattr(self.yolo_model.model, "names")
            else self.yolo_model.names
        )

        # Camera intrinsics
        self.cx = 320
        self.cy = 240
        self.fx = 426.7
        self.fy = 428.6
    
        self.create_timer(1.0, self.get_obj_pose)

        self.get_logger().info("PERCEPTRON started")

    def callback(self, rgb_msg, depth_msg):
        try:
            self.rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {e}")
            return
        
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")
            return

        if self.visual:
            depth_norm = cv2.normalize(self.depth_img, None, 0, 255, cv2.NORM_MINMAX)
            depth_norm = depth_norm.astype(np.uint8)
            depth_colormap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

            cv2.imshow("RGB Image", self.rgb_img)
            cv2.imshow("Depth Heatmap", depth_colormap)
            cv2.waitKey(1)

    def get_obj_pose(self):
        if self.rgb_img is None or self.depth_img is None:
            return

        results = self.yolo_model(self.rgb_img, imgsz=640, conf=0.25, verbose=False)
        res = results[0]

        boxes, classes = [], []

        if hasattr(res, "boxes") and res.boxes is not None:
            for box in res.boxes:
                xyxy = box.xyxy.cpu().numpy().flatten()
                cls = int(box.cls.cpu().numpy().item())
                boxes.append(xyxy)
                classes.append(cls)

        count = 0
        if boxes:
            obj_msg = Object()

            for (xyxy, cls) in zip(boxes, classes):
                x1, y1, x2, y2 = map(int, xyxy)
                obj_name = self.names.get(int(cls), f"class_{cls}")

                cx_box = int((x1 + x2) / 2)
                cy_box = int((y1 + y2) / 2)

                Z = float(self.depth_img[cy_box][cx_box])

                if Z == 0 or Z > 1000 or obj_name == f"person" or obj_name == f"dining table":
                    continue

                X = (cx_box - self.cx) * Z / self.fx
                Y = (cy_box - self.cy) * Z / self.fy

                # Convert mm → meters
                Xm = X / 1000.0
                Ym = Y / 1000.0
                Zm = Z / 1000.0

                # Publish TF transform camera_link -> detected_object
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "camera_link"
                t.child_frame_id = obj_name

                t.transform.translation.x = Xm
                t.transform.translation.y = Ym
                t.transform.translation.z = Zm

                # No rotation → identity quaternion
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0

                self.tf_broadcaster.sendTransform(t)

                # self.get_logger().info(
                #     f"TF Published: {frame_name} at {Xm:.3f}, {Ym:.3f}, {Zm:.3f}"
                # )
                
                point_msg = Point()

                pose = self.pose_in_base(obj_name)

                if pose is not None:
                    point_msg.x, point_msg.y, point_msg.z = pose
                
                if point_msg.x <= 0.0: # right side
                    obj_msg.obj_right.append(obj_name)
                    obj_msg.obj_pose_right.append(point_msg)
                
                else:
                    obj_msg.obj_left.append(obj_name)
                    obj_msg.obj_pose_left.append(point_msg)

                self.get_logger().info(
                    f"{obj_name} : {point_msg.x}, {point_msg.y}, {point_msg.z}"
                )

            self.obj_pub.publish(obj_msg)

    def pose_in_base(self, object_name: str):

        try:
            # Lookup transform: base_link → object_name
            trans = self.tf_buffer.lookup_transform(
                "base",          # target frame
                object_name,          # source frame
                rclpy.time.Time()     # latest available
            )

            x = trans.transform.translation.x
            y = trans.transform.translation.y
            z = trans.transform.translation.z

            return x, y, z

        except Exception as e:
            self.get_logger().warn(f"[TF Lookup Failed] {object_name}: {str(e)}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = RGBDViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
