#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ColorFilter:
    def __init__(self):
        rospy.init_node('color_filter_node', anonymous=True)

        self.target_color = rospy.get_param('~color', 'красный').lower()

        self.bridge = CvBridge()

        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.mask_pub = rospy.Publisher('/color_mask', Image, queue_size=10)
        self.original_pub = rospy.Publisher('/original_image', Image, queue_size=10)

        rospy.loginfo(f"Цвет для фильтрации: {self.target_color}")
        rospy.spin()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower, upper = self.get_color_bounds(self.target_color)
        mask = cv2.inRange(hsv, lower, upper)

        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, encoding='mono8'))
        self.original_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding='bgr8'))
        cv2.imshow("Filtered Image", mask)
        cv2.waitKey(1)

    def get_color_bounds(self, color):
        if color == 'красный':
            return np.array([0, 100, 100]), np.array([10, 255, 255])
        elif color == 'зеленый':
            return np.array([45, 100, 100]), np.array([75, 255, 255])
        elif color == 'синий':
            return np.array([100, 100, 100]), np.array([130, 255, 255])
        else:
            rospy.logwarn("Неизвестный цвет.")
            return np.array([0, 100, 100]), np.array([10, 255, 255])

if __name__ == '__main__':
    try:
        ColorFilter()
    except rospy.ROSInterruptException:
        pass
