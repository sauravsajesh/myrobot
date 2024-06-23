#!/usr/bin/env python
# src/motion_detector.py

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class MotionDetector:
    def __init__(self):
        rospy.init_node('motion_detector')
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.previous_frame = None

    def image_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        if self.previous_frame is None:
            self.previous_frame = gray
            return

        frame_delta = cv2.absdiff(self.previous_frame, gray)
        thresh = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.dilate(thresh, None, iterations=2)
        contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        movement_detected = any(cv2.contourArea(contour) > 500 for contour in contours)
        self.handle_movement(movement_detected)
        self.previous_frame = gray

    def handle_movement(self, movement_detected):
        twist = Twist()
        if movement_detected:
            rospy.loginfo("Movement detected! Stopping the robot.")
            twist.linear.x = 0
            twist.angular.z = 0
        self.pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    motion_detector = MotionDetector()
    motion_detector.run()
