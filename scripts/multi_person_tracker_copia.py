#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
from collections import defaultdict
from crop_and_wrap import crop_and_wrap


class MultiPersonTrackerNode(Node):
    def __init__(self):
        super().__init__('multi_person_tracker')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, 'stitched_image', self.image_callback, 10)
        self.pub_annotated = self.create_publisher(Image, 'multi_object_tracker', 10)

        self.track_history = defaultdict(lambda: [])

        self.declare_parameter(
            'object_tracked',
            'full_image',
            descriptor=ParameterDescriptor(
                name='object_tracked',
                type=ParameterType.PARAMETER_STRING,
                description="Track ID to crop and publish. Use 'full_image' to disable cropping."
            )
        )

        self.model = YOLO("yolo11m.pt")
        self.model.tracker = "bytetrack.yaml"
        self.model.persist = True

        self.get_logger().info("YOLOv8 Multi-Object Tracking node started.")

    def image_callback(self, msg):
        object_tracked = self.get_parameter('object_tracked').get_parameter_value().string_value
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model.track(
            frame,
            classes=[0],
            conf=0.5,
            iou=0.5,
            persist=True,
            verbose=False
        )

        annotated = frame  # fallback if no detections

        if results[0].boxes and results[0].boxes.id is not None:
            boxes = results[0].boxes.xywh.cpu()
            ids = results[0].boxes.id.int().cpu().tolist()

            if object_tracked != 'full_image':
                try:
                    target_id = int(object_tracked)
                    for box, tid in zip(boxes, ids):
                        if tid == target_id:
                            cx, cy, w, h = box.tolist()
                            x = int(cx - w / 2)
                            y = int(cy - h / 2)
                            bbox_xywh = (x, y, int(w), int(h))
                            cropped, (x1_draw, y1, x2_draw, y2) = crop_and_wrap(frame, bbox_xywh,pad_ratio=0.1)
                            annotated = cropped
            
                            break
                    else:
                        self.get_logger().warn(f"Track ID {target_id} not found in this frame.")
                except ValueError:
                    self.get_logger().warn(f"Invalid track ID parameter: {object_tracked}")
            else:
                annotated = results[0].plot()
        else:
            self.get_logger().warn("No detections in frame.")

        msg_out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        msg_out.header = msg.header
        self.pub_annotated.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = MultiPersonTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
