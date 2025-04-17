#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

class MotionDetector:
    def __init__(self):
        rospy.init_node('motion_detector', anonymous=True)
        self.bridge = CvBridge()
        self.prev_frame = None
        self.motion_pub = rospy.Publisher('/usb_cam/motion_detection', Bool, queue_size=10)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        rospy.spin()

    def callback(self, msg):

        curr_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(curr_frame, cv2.COLOR_BGR2GRAY)

        if self.prev_frame is None:
            self.prev_frame = gray
            return

        diff = cv2.absdiff(self.prev_frame, gray)


        _, thresh = cv2.threshold(diff, 25, 255, cv2.THRESH_BINARY)

        motion_pixels = np.sum(thresh > 0)

        motion_detected = motion_pixels > 5000

        self.motion_pub.publish(Bool(data=motion_detected))

        self.prev_frame = gray

        cv2.imshow("Motion Detection", thresh)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        MotionDetector()
    except rospy.ROSInterruptException:
        pass
