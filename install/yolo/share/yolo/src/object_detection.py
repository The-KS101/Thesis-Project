#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory

class YOLOv8ObjectDetector(Node):
    def __init__(self):
        super().__init__('yolov8_object_detector')
        self.bridge = CvBridge()
        package_dir = get_package_share_directory('yolo')  # Replace with your package name
        weights_path = os.path.join(package_dir, 'weights', 'best.pt')

        self.model = YOLO(weights_path)  # Path to your trained YOLOv8 weights
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Image, '/yolov8_detected_image', 10)

    def listener_callback(self, msg):
            self.get_logger().info('Receiving video frame')
            current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            results = self.model(current_frame)
            boxes = results[0].boxes

            for box in boxes:
                # Get the coordinates
                xyxy = box.xyxy[0].tolist()
                x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])

                # Get the class ID and confidence
                cls = int(box.cls[0])
                conf = float(box.conf[0])  # Convert tensor to float

                # Get the label
                label = f'{self.model.names[cls]} {conf:.2f}'

                # Draw the bounding box and label on the image
                cv2.rectangle(current_frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(current_frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # Convert the annotated image back to ROS Image message
            annotated_msg = self.bridge.cv2_to_imgmsg(current_frame, "bgr8")

            # Publish the annotated image
            self.publisher_.publish(annotated_msg)
def main(args=None):
    rclpy.init(args=args)
    yolov8_object_detector = YOLOv8ObjectDetector()
    rclpy.spin(yolov8_object_detector)
    yolov8_object_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()