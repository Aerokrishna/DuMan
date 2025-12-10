#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from ultralytics import YOLO

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

        self.rgb_img = None
        self.depth_img = None
        self.visual = True

        self.yolo_model = YOLO("yolov8n.pt")
        self.names = self.yolo_model.model.names if hasattr(self.yolo_model, "model") and hasattr(self.yolo_model.model, "names") else {}
        if not self.names and hasattr(self.yolo_model, "names"):
            self.names = self.yolo_model.names

        self.cx = 320
        self.cy = 240
        self.fx = 426.7
        self.fy = 428.6
    
        self.create_timer(1.0, self.get_obj_pose)

        self.get_logger().info("PERCEPTRON")

    def callback(self, rgb_msg, depth_msg):
        try:
            # Convert RGB image
            self.rgb_img = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {e}")
            return
        
        # Convert Depth Image (uint16 → millimeters or meters)
        try:
            self.depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")
            return

        if self.visual:
            # Normalize depth for visualization
            depth_norm = cv2.normalize(self.depth_img, None, 0, 255, cv2.NORM_MINMAX)
            depth_norm = depth_norm.astype(np.uint8)
            depth_colormap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

            # Show RGB image
            cv2.imshow("RGB Image", self.rgb_img)
            cv2.imshow("Depth Heatmap", depth_colormap)
            cv2.waitKey(1)

    def get_obj_pose(self):
        if self.rgb_img is not None and self.depth_img is not None:
            results = self.yolo_model(self.rgb_img, imgsz=640, conf=0.25, verbose=False)
            res = results[0]

            boxes, classes = [], []
            if hasattr(res, "boxes") and res.boxes is not None:
                for box in res.boxes:
                    xyxy = box.xyxy.cpu().numpy().flatten()
                    conf = float(box.conf.cpu().numpy().item())
                    cls = int(box.cls.cpu().numpy().item())
                    boxes.append(xyxy)
                    classes.append(cls)

            # Print detected object info to terminal
            if boxes:
                for (xyxy, cls) in zip(boxes, classes):
                    x1, y1, x2, y2 = map(int, xyxy)
                    obj_name = self.names.get(int(cls), f"Class {cls}")
                    cx_box, cy_box = int((x1 + x2) / 2), int((y1 + y2) / 2)

                    Z = self.depth_img[cy_box][cx_box]
                    X = (cx_box - self.cx) * Z / self.fx
                    Y = (cy_box - self.cy) * Z / self.fy

                    if Z!=0 and obj_name!="person":
                        self.get_logger().info(f" {obj_name} POSE : {X} {Y} {Z}")

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
