#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from crop_and_wrap import crop_and_wrap

class YoloTrackerNode(Node):
    def __init__(self):
        super().__init__('yolo_tracker_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, 'stitched_image', self.image_callback, 10)

        self.pub_crop = self.create_publisher(Image, 'object_crop', 10)
        self.pub_annotated = self.create_publisher(Image, 'object_annotated', 10)

        package_path = get_package_share_directory('tracker_360')
        model_path = os.path.join(package_path, 'yolov11n-face.pt')
        self.model = YOLO(model_path)
        self.tracker = None
        self.tracking_initialized = False
        self.get_logger().info("YOLO + Tracker node ready.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if not self.tracking_initialized:
            results = self.model(frame)
            boxes = results[0].boxes

            if len(boxes) == 0:
                self.get_logger().info("No object detected.")
                return

            xyxy = boxes.xyxy[0].cpu().numpy().astype(int)
            x1, y1, x2, y2 = xyxy
            w, h = x2 - x1, y2 - y1

            self.tracker = cv2.TrackerCSRT_create()
            self.tracker.init(frame, (x1, y1, w, h))
            self.tracking_initialized = True
            self.get_logger().info(f"Tracker initialized with bbox: {(x1, y1, w, h)}")
        else:
            success, bbox = self.tracker.update(frame)
            if not success:
                self.get_logger().warn("Tracking lost. Reinitializing.")
                self.tracking_initialized = False
                return

            cropped, (x1_draw, y1, x2_draw, y2) = crop_and_wrap(frame, bbox, pad_ratio=0.6)

            # Annotate and publish
            annotated = frame.copy()
            cv2.rectangle(annotated, (x1_draw, y1), (x2_draw, y2), (0, 255, 0), 2)

            crop_msg = self.bridge.cv2_to_imgmsg(cropped, encoding="bgr8")
            crop_msg.header = msg.header
            self.pub_crop.publish(crop_msg)

            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            annotated_msg.header = msg.header
            self.pub_annotated.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
