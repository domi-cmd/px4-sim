#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/yolo/image_annotated', 10)
        self.create_subscription(Image, '/camera/image_raw', self.cb, 10)

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        t0 = time.perf_counter()
        results = self.model.predict(frame, conf=0.45, verbose=False)
        ms = (time.perf_counter() - t0) * 1000
        annotated = results[0].plot()  # draws boxes automatically
        self.get_logger().info(f'{len(results[0].boxes)} detections | {ms:.1f}ms')
        self.pub.publish(self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))

rclpy.init()
rclpy.spin(YoloNode())