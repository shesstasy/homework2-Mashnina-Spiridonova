#!/usr/bin/env python3
# encoding: utf-8

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from typing import Final

# Константы
ROS_NODE_NAME: Final[str] = "subscriber_elp.py"
ROS_SUB_TOPIC: Final[str] = "/usb_cam/image_raw"
ROS_PUB_TOPIC: Final[str] = "/usb_cam/image_color"

def image_callback(msg: Image, cv_bridge: CvBridge, publisher: rospy.Publisher) -> None:
    try:

        yuyv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="yuv422")
        rgb_image = cv2.cvtColor(yuyv_image, cv2.COLOR_YUV2RGB_YUYV)
        rgb_msg = cv_bridge.cv2_to_imgmsg(rgb_image, encoding="mono8")
        rgb_msg.header = msg.header  # Сохраняем заголовок
        
        # Публикация
        publisher.publish(rgb_msg)
        
        rospy.loginfo_once(f"Изображение преобразовано из {msg.encoding} в rgb8")
        
    except Exception as e:
        rospy.logerr(f"Ошибка обработки изображения: {str(e)}")

def main() -> None:

    rospy.init_node(ROS_NODE_NAME, anonymous=True)
    

    cv_bridge = CvBridge()
    

    publisher = rospy.Publisher(ROS_PUB_TOPIC, Image, queue_size=10)
    

    rospy.loginfo(f"Подписка на {ROS_SUB_TOPIC}, публикация в {ROS_PUB_TOPIC}")
    

    rospy.Subscriber(ROS_SUB_TOPIC, Image, lambda msg: image_callback(msg, cv_bridge, publisher), queue_size=10)

    rospy.spin()

if __name__ == "__main__":
    main()
