#!/usr/bin/env python3

import math

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

import rclpy
from rclpy.node import Node

import tf2_ros
import tf2_geometry_msgs # needs to be imported to make posestamped to tf2_posestamped conversion


class PersonGoalTf(Node):

    def __init__(self):
        super().__init__('person_goal_tf')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.ns = "person"
        self.marker.id = 0
        self.marker.type = 3
        self.marker.action = 0
        self.marker.scale.x = 0.5
        self.marker.scale.y = 0.5
        self.marker.scale.z = 0.5
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0

        self._person_pub_sub = self.create_subscription(
            PoseStamped,
            '/person_goal',
            self.person_goal_callback,
            10)
        self._person_pub_sub # avoid unused variable warning

        self.person_pos_marker_pub = self.create_publisher(Marker, '/person_marker', 1)
        self.person_pos_pub = self.create_publisher(PoseStamped, '/goal_update', 1)

    def person_goal_callback(self, data):
        try:
            # It is important to wait for the listener to start listening. (rclpy.Duration)
            output_pose_stamped = self.tf_buffer.transform(data, "map", rclpy.duration.Duration(seconds=0.001))

            self.marker.pose.position.x = output_pose_stamped.pose.position.x
            self.marker.pose.position.y = output_pose_stamped.pose.position.y
            self.marker.pose.position.z = output_pose_stamped.pose.position.z
            self.marker.pose.orientation.x = output_pose_stamped.pose.orientation.x
            self.marker.pose.orientation.y = output_pose_stamped.pose.orientation.y
            self.marker.pose.orientation.z = output_pose_stamped.pose.orientation.z
            self.marker.pose.orientation.w = output_pose_stamped.pose.orientation.w
            
            self.person_pos_marker_pub.publish(self.marker)
            self.person_pos_pub.publish(output_pose_stamped)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise


if __name__ == '__main__':
    rclpy.init(args=None)
    person_goal = PersonGoalTf()
    rclpy.spin(person_goal)
    rclpy.shutdown()