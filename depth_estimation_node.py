#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Point
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np

def depth_estimation():
    rospy.init_node('depth_estimation_node', anonymous=True)

    depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    pattern1_sub = message_filters.Subscriber('/pattern1_coordinates', Point)
    pattern2_sub = message_filters.Subscriber('/pattern2_coordinates', Point)

    ts = message_filters.ApproximateTimeSynchronizer([depth_sub, pattern1_sub, pattern2_sub], 10, 0.1)
    ts.registerCallback(callback)

    rospy.spin()

def callback(depth_msg, pattern1_msg, pattern2_msg):
    bridge = CvBridge()
    try:
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    z1 = depth_image[int(pattern1_msg.y), int(pattern1_msg.x)]
    z2 = depth_image[int(pattern2_msg.y), int(pattern2_msg.x)]

    point1_3d = compute_3d_point(pattern1_msg.x, pattern1_msg.y, z1)
    point2_3d = compute_3d_point(pattern2_msg.x, pattern2_msg.y, z2)

    pub_point1_3d.publish(point1_3d)
    pub_point2_3d.publish(point2_3d)

def compute_3d_point(x, y, z):
    point = PointStamped()
    point.header.stamp = rospy.Time.now()
    point.point.x = x
    point.point.y = y
    point.point.z = z
    return point

if __name__ == '__main__':
    pub_point1_3d = rospy.Publisher('/pattern1_3d', PointStamped, queue_size=10)
    pub_point2_3d = rospy.Publisher('/pattern2_3d', PointStamped, queue_size=10)

    depth_estimation()
