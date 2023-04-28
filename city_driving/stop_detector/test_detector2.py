#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def publish_image():
    # Initialize ROS node
    rospy.init_node('image_publisher', anonymous=True)

    # Initialize ROS publisher
    pub = rospy.Publisher('/zed/zed_node/rgb/image_rect_color', Image, queue_size=10)

    # Initialize OpenCV bridge
    bridge = CvBridge()

    # Load image
    img = cv2.imread('../../media/city_driving.png')
    print(img.shape)
    # Convert image to ROS message
    img_msg = bridge.cv2_to_imgmsg(img, encoding='bgr8')

    # Publish image message
    pub.publish(img_msg)

    # Spin once to publish the message
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_image()
    except rospy.ROSInterruptException:
        pass