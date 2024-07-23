#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2
import sys

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO('yolov8n.pt')

        self.yolov8_inference = Yolov8Inference()

        self._subscriber_color_camera_raw = self.create_subscription(
            Image,
            'color/image_raw',
            self.camera_callback,
            10)
        self._subscriber_color_camera_raw

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    def camera_callback(self, data):
        
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls 
                self.inference_result.class_name = self.model.names[int(c)]
                self.inference_result.x1 = int(b[0])
                self.inference_result.y1 = int(b[1])
                self.inference_result.x2 = int(b[2])
                self.inference_result.y2 = int(b[3])
                self.yolov8_inference.yolov8_inference.append(self.inference_result)
        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)

        self.img_pub.publish(img_msg)
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
