#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from concurrent.futures import ThreadPoolExecutor
from panoramic_projections import precompute_stereo_luts, apply_stereo_luts_parallel, stereo_bboxes_to_panorama
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator

class PanoramaProjectionPublisher(Node):
    def __init__(self):
        super().__init__('panorama_projection_publisher')

        self.bridge = CvBridge()
        self.stereo_img_size = (448, 448)
        self.FOV = (180, 180)
        self.pano_shape = (960, 1920)
        self.conf_thres = 0.7

        self.subscription = self.create_subscription(
            Image, 'stitched_image', self.image_callback, 5)
        self.annotated_pub = self.create_publisher(Image, 'annotated_panorama', 5)

        self.luts = precompute_stereo_luts(
            pano_shape=self.pano_shape,
            output_size=self.stereo_img_size,
            FOV=self.FOV,
            pitch=90
        )

        self.yolo_model = YOLO("yolo11m.pt")
        self.get_logger().info('Started with native YOLOv8 model')

    def image_callback(self, msg):
        pano_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        proj_msgs = apply_stereo_luts_parallel(pano_img, self.luts, self.bridge, msg.header)

        projections = []
        for i, proj_msg in enumerate(proj_msgs):
            img = self.bridge.imgmsg_to_cv2(proj_msg, desired_encoding='bgr8')
            scale = img.shape[1] / self.stereo_img_size[0]
            projections.append({'image': img, 'yaw': self.luts[i]['yaw'], 'pitch': self.luts[i]['pitch'], 'scale': scale})

        detections = self.detect_on_projections(projections)

        if not any(d['image_detections'] for d in detections):
            self.get_logger().warn('No detections found in any projection')

        annotated,_ = stereo_bboxes_to_panorama(
            detections_with_meta=detections,
            pano_img=pano_img.copy(),
            stereo_img_size=self.stereo_img_size,
            FOV=self.FOV,
            model = self.yolo_model
        
        )

        out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out.header = msg.header
        self.annotated_pub.publish(out)

    def detect_on_projections(self, projections):
        results = []
        for p in projections:
            meta = {
                'yaw': p['yaw'],
                'pitch': p['pitch'],
                'scale': 1.0  # porque no escalamos la imagen
            }
            dets = []
            res = self.yolo_model.predict(
                p['image'],
                imgsz=p['image'].shape[0],  # tamaño real de la imagen
                conf=self.conf_thres,
                verbose=False
            )[0]
            if res.boxes is not None:
                for box, conf, cls in zip(res.boxes.xywh, res.boxes.conf, res.boxes.cls):
                    x, y, w, h = box.tolist()
                    dets.append({
                        'box': [x, y, w, h],
                        'confidence': float(conf),
                        'class_id': int(cls),
                        'scale': 1.0
                    })
            results.append({
                'image_detections': dets,
                'yaw': meta['yaw'],
                'pitch': meta['pitch']
            })
        return results
def main(args=None):
    rclpy.init(args=args)
    node = PanoramaProjectionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

