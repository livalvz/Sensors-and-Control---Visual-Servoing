#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def pattern_recognition():
    rospy.init_node('pattern_recognition_node', anonymous=True)
    image_topic = '/camera/color/image_raw'
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.spin()

def image_callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    # Example pattern detection (replace with actual logic)
    x1, y1 = detect_pattern(cv_image, pattern1_template)
    x2, y2 = detect_pattern(cv_image, pattern2_template)

    # Publish the coordinates
    point1 = Point(x=x1, y=y1, z=0)
    point2 = Point(x=x2, y=y2, z=0)
    pub_pattern1.publish(point1)
    pub_pattern2.publish(point2)

def detect_pattern(image, template):
    # Placeholder pattern detection logic
    x, y = 0, 0
    return x, y

if __name__ == '__main__':
    pub_pattern1 = rospy.Publisher('/pattern1_coordinates', Point, queue_size=10)
    pub_pattern2 = rospy.Publisher('/pattern2_coordinates', Point, queue_size=10)

    pattern1_template = cv2.imread('path_to_pattern1_template.png', 0)
    pattern2_template = cv2.imread('path_to_pattern2_template.png', 0)

    pattern_recognition()
