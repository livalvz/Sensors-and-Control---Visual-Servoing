#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class PatternRecognitionNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pattern_recognition_node', anonymous=True)
        self.bridge = CvBridge()
        
        # Publishers
        self.pub_pattern1 = rospy.Publisher('/pattern1_coordinates', Point, queue_size=10)
        self.pub_pattern2 = rospy.Publisher('/pattern2_coordinates', Point, queue_size=10)
        
        # Subscriber
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        rospy.loginfo("Pattern recognition node started.")
        
        # Create OpenCV windows for visualization
        cv2.namedWindow('Checkerboard Detection', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Circle Grid Detection', cv2.WINDOW_NORMAL)
        
    def detect_pattern(self, image, pattern_type='checkerboard'):
        try:
            if pattern_type == 'checkerboard':
                # Pattern size for your checkerboard: 9x6 inner corners
                pattern_size = (9, 6)
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
    
                if ret:
                    # Refine corner locations for better accuracy
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    # Return the coordinates of the first corner
                    x, y = corners2[0][0]
                    rospy.loginfo(f"Checkerboard detected at: ({x}, {y})")
                    return x, y, corners2
                else:
                    rospy.logdebug("Checkerboard pattern not found.")
            elif pattern_type == 'circle':
                # Pattern size for your circle grid: 4x4 grid
                pattern_size = (4, 4)
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                # Use findCirclesGrid for detecting the circle pattern
                ret, centers = cv2.findCirclesGrid(gray, pattern_size, cv2.CALIB_CB_SYMMETRIC_GRID)
    
                if ret:
                    x, y = centers[0][0]
                    rospy.loginfo(f"Circle grid detected at: ({x}, {y})")
                    return x, y, centers
                else:
                    rospy.logdebug("Circle grid pattern not found.")
        except cv2.error as e:
            rospy.logerr(f"OpenCV Error in detect_pattern: {e}")
        except Exception as e:
            rospy.logerr(f"Unexpected error in detect_pattern: {e}")
    
        return None  # Return None if pattern not found
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return
    
        # Copy of the image for visualization
        checkerboard_image = cv_image.copy()
        circle_grid_image = cv_image.copy()
    
        # Detect checkerboard pattern
        result = self.detect_pattern(cv_image, pattern_type='checkerboard')
        if result is not None:
            x1, y1, corners2 = result
            point1 = Point(x=x1, y=y1, z=0)
            self.pub_pattern1.publish(point1)
            # Visualize the detected checkerboard pattern
            cv2.drawChessboardCorners(checkerboard_image, (9, 6), corners2, True)
            cv2.imshow('Checkerboard Detection', checkerboard_image)
        else:
            rospy.logdebug("Checkerboard pattern not detected; not publishing coordinates.")
            cv2.imshow('Checkerboard Detection', checkerboard_image)
    
        # Detect circle grid pattern
        result = self.detect_pattern(cv_image, pattern_type='circle')
        if result is not None:
            x2, y2, centers = result
            point2 = Point(x=x2, y=y2, z=0)
            self.pub_pattern2.publish(point2)
            # Visualize the detected circle grid pattern
            cv2.drawChessboardCorners(circle_grid_image, (4, 4), centers, True)
            cv2.imshow('Circle Grid Detection', circle_grid_image)
        else:
            rospy.logdebug("Circle grid pattern not detected; not publishing coordinates.")
            cv2.imshow('Circle Grid Detection', circle_grid_image)
    
        # Wait for a key press for a short time to allow OpenCV to update windows
        cv2.waitKey(1)
    
    def run(self):
        rospy.spin()
        # Destroy OpenCV windows when ROS node is shut down
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = PatternRecognitionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

