#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

def publish_image():
    # Initialize ROS node
    rospy.init_node('image_publisher', anonymous=True)

    # Initialize ROS publisher
    img_pub = rospy.Publisher('/zed/zed_node/rgb/image_rect_color', Image, queue_size=10)
    depth_pub = rospy.Publisher('/zed/zed_node/depth/depth_registered', Image, queue_size=10)

    # Initialize OpenCV bridge
    bridge = CvBridge()

    # Load image
    img = cv2.imread('../../media/stop_sign.jpg')
    height, width = img.shape[:2]
    # Convert image to ROS message
    img_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')

    while not rospy.is_shutdown():
        depth_img = np.random.randint(0, 1000, size=(480, 640), dtype=np.uint16)
        depth_msg = bridge.cv2_to_imgmsg(depth_img, '16UC1')
        # Publish image message
        img_pub.publish(img_msg)
        depth_pub.publish(depth_msg)
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass