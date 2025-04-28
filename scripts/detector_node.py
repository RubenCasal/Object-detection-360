#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.subscription = self.create_subscription(
            Image,
            'stitched_image',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Image, 'stitched_image_annotated', 10)
        self.bridge = CvBridge()
        self.model = YOLO('yolo11m.pt')  # Cambia a yolov8n-face.pt para detección facial
        self.get_logger().info('YOLOv8 detector loaded and running.')

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        results = self.model(frame)
        annotated = results[0].plot()  # Imagen con cajas dibujadas
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='rgb8')
        out_msg.header = msg.header
        self.publisher.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
