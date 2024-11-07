#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, PoseStamped
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class VisualServoingNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('visual_servoing_node', anonymous=True)
        self.bridge = CvBridge()

        # Publishers
        self.pub_pattern_coords = rospy.Publisher('/pattern_coordinates', Point, queue_size=10)
        self.pub_pattern_pose = rospy.Publisher('/pattern_pose', PoseStamped, queue_size=10)
        self.pub_error = rospy.Publisher('/visual_servoing/error', Point, queue_size=10)

        # Camera calibration parameters (replace with your actual parameters)
        # Intrinsic parameters extracted from your 'rs-enumerate-devices -c' output
        self.camera_matrix = np.array([
            [910.246643066406, 0.0, 659.972534179688],
            [0.0, 909.000122070312, 369.965209960938],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        # Distortion coefficients (assuming zero distortion as per your output)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)  # [k1, k2, p1, p2, k3]

        # Pattern settings
        self.checkerboard_size = (7, 7)  # Number of inner corners per chessboard row and column
        self.square_size = 0.03  # Size of a square in meters (3 cm squares)

        # Desired pose relative to the pattern (adjust as needed)
        self.desired_tvec = np.array([[0.0], [0.0], [0.5]])  # 50 cm in front of the pattern
        self.desired_rvec = np.array([[0.0], [0.0], [0.0]])  # Facing directly at the pattern

        # Subscriber to the camera image topic
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        rospy.loginfo("Visual servoing node started.")

        # Create OpenCV window for visualization
        cv2.namedWindow('Visual Servoing', cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Copy image for visualization
        display_image = cv_image.copy()

        # Detect and process checkerboard pattern
        display_image = self.detect_and_process_pattern(cv_image, display_image)

        # Show visualization
        cv2.imshow('Visual Servoing', display_image)
        cv2.waitKey(1)

    def detect_and_process_pattern(self, original_image, display_image):
        # Convert image to grayscale
        gray = cv2.cvtColor(original_image, cv2.COLOR_BGR2GRAY)

        # Find checkerboard corners
        ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)

        if ret:
            rospy.loginfo("Checkerboard pattern found.")

            # Refine corner locations
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # Publish 2D coordinates of the first corner
            x2d, y2d = corners_refined[0][0]
            point2d = Point(x=float(x2d), y=float(y2d), z=0.0)
            self.pub_pattern_coords.publish(point2d)

            # Draw the detected corners
            cv2.drawChessboardCorners(display_image, self.checkerboard_size, corners_refined, ret)

            # Prepare object points
            objp = np.zeros((self.checkerboard_size[0] * self.checkerboard_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:self.checkerboard_size[0], 0:self.checkerboard_size[1]].T.reshape(-1, 2)
            objp *= self.square_size

            # Solve PnP to get rotation and translation vectors
            success, rvec, tvec = cv2.solvePnP(objp, corners_refined, self.camera_matrix, self.dist_coeffs)

            if success:
                # Create and publish pose message
                pose = self.create_pose_stamped(rvec, tvec, frame_id='camera_color_optical_frame')
                self.pub_pattern_pose.publish(pose)
                rospy.loginfo("Pattern pose published.")

                # Compute and publish position error
                error_tvec = tvec - self.desired_tvec
                error_point = Point(x=float(error_tvec[0][0]), y=float(error_tvec[1][0]), z=float(error_tvec[2][0]))
                self.pub_error.publish(error_point)

                # Calculate error norm
                error_norm = np.linalg.norm(error_tvec)
                rospy.loginfo(f"Position error (norm): {error_norm:.3f} meters")

                # Display error on image
                cv2.putText(display_image, f"Position Error: {error_norm:.2f} m", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                rospy.logwarn("Failed to solve PnP for checkerboard.")
        else:
            rospy.logdebug("Checkerboard pattern not found.")
            cv2.putText(display_image, "Checkerboard pattern not found", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        return display_image

    def create_pose_stamped(self, rvec, tvec, frame_id):
        # Convert rotation vector to quaternion
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame_id

        pose.pose.position.x = float(tvec[0][0])
        pose.pose.position.y = float(tvec[1][0])
        pose.pose.position.z = float(tvec[2][0])

        pose.pose.orientation.x = float(quaternion[0])
        pose.pose.orientation.y = float(quaternion[1])
        pose.pose.orientation.z = float(quaternion[2])
        pose.pose.orientation.w = float(quaternion[3])

        return pose

    def rotation_matrix_to_quaternion(self, R):
        # Convert rotation matrix to quaternion
        q = np.empty((4, ), dtype=np.float64)
        trace = np.trace(R)
        if trace > 0.0:
            s = 2.0 * np.sqrt(trace + 1.0)
            q[3] = 0.25 * s
            q[0] = (R[2, 1] - R[1, 2]) / s
            q[1] = (R[0, 2] - R[2, 0]) / s
            q[2] = (R[1, 0] - R[0, 1]) / s
        else:
            if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s

        # Normalize quaternion
        q_norm = np.linalg.norm(q)
        q = q / q_norm
        # Return quaternion as [x, y, z, w]
        return q

    def run(self):
        rospy.spin()
        # Destroy OpenCV windows when ROS node is shut down
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = VisualServoingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
g
