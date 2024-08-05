#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import math
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2
import sys

from yolov8_msgs.msg import InferencePosition
from yolov8_msgs.msg import Yolov8Inference
from yolov8_msgs.msg import InferenceResult
from geometry_msgs.msg import PoseStamped


bridge = CvBridge()

class InferencePositionCalc(Node):

    def __init__(self):
        super().__init__('inference_position')

        self.inference_position = InferencePosition()

        self.person_goal = PoseStamped()
        self.person_goal.header.stamp = self.get_clock().now().to_msg()
        self.person_goal.header.frame_id = "camera_link"
        self.person_goal.pose.orientation.x = 0.0
        self.person_goal.pose.orientation.y = 0.0
        self.person_goal.pose.orientation.z = 0.0
        self.person_goal.pose.orientation.w = 1.0
        
        # realsense intrinsics values from camera_info topic
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = 1280
        self.intrinsics.height = 720
        self.intrinsics.ppx = 640
        self.intrinsics.ppy = 360
        self.intrinsics.fx = 695.9951171875
        self.intrinsics.fy = 695.9951171875

        self.inferences_center = []

        self._subscriber_inference_result = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.inference_result_callback,
            10)
        self._subscriber_inference_result # avoid unused variable warning

        self._subscriber_depth_camera_raw = self.create_subscription(
            Image,
            'depth/image_raw',
            self.image_depth_callback,
            10)
        self._subscriber_depth_camera_raw # avoid unused variable warning

        self.inference_pos_pub = self.create_publisher(InferencePosition, "/inference_position", 1)
        self.person_goal_pub = self.create_publisher(PoseStamped, "/person_goal", 1)

    def image_depth_callback(self, data):
        try:
            cv_image = bridge.imgmsg_to_cv2(data, data.encoding)
            if len(self.inferences_center) > 0:
                for i in self.inferences_center:
                    inference_center = (i[1], i[2])
                    dist = cv_image[int(inference_center[1]), int(inference_center[0])] * 0.001
                    self.inference_position.class_name = i[0]
                    self.inference_position.y = -(dist*(i[1] - self.intrinsics.ppx)/self.intrinsics.fx - 0.035)
                    self.inference_position.z = -(dist*(i[2] - self.intrinsics.ppy)/self.intrinsics.fy)
                    self.inference_position.x = float(dist)

                    if i[0] == "person":
                        self.person_goal.pose.position.x = self.inference_position.x
                        self.person_goal.pose.position.y = self.inference_position.y
                        self.person_goal.pose.position.z = self.inference_position.z
                        self.person_goal_pub.publish(self.person_goal)

                    self.inference_pos_pub.publish(self.inference_position)

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return
        
    # gets inference center    
    def inference_result_callback(self, data): 
        self.inferences_center.clear()
        for i in data.yolov8_inference:
            inference = []
            inference_class_name = i.class_name
            inference_center_x = (i.x1 + i.x2)/2
            inference_center_y = (i.y1 + i.y2)/2
            inference.append(inference_class_name)
            inference.append(inference_center_x)
            inference.append(inference_center_y)
            self.inferences_center.append(inference)
        

if __name__ == '__main__':
    rclpy.init(args=None)
    inference_position = InferencePositionCalc()
    rclpy.spin(inference_position)
    rclpy.shutdown()
