#!/usr/bin/env python2

import cv2
import rospy

import numpy as np
from sensor_msgs.msg import Image
from detector import StopSignDetector
from final_challenge2023.msg import DetectedObject

class SignDetector:
    def __init__(self):
        self.detector = StopSignDetector()

        self.stopsign_present = False
        self.left, self.top, self.right, self.bottom = 0, 0, 0, 0

        self.depth = None

        scan_topic = rospy.get_param("~stopsign_topic", "/stopsign")
        self.publisher = rospy.Publisher(scan_topic, DetectedObject, queue_size=20)
        self.rgb_subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.color_callback)
        self.depth_subscriber = rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, self.depth_callback)


    def color_callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:, :, :3]
        rgb_img = bgr_img[:, :, ::-1]
        width, height = len(rgb_img[0]), len(rgb_img)

        self.stopsign_present, bounding_box = self.detector.predict(rgb_img)
        self.left, self.top, self.right, self.bottom = bounding_box

        if self.depth is None:
            self.depth = np.ones([height, width]) * 1000.0

        self.publish_stopsign_message()

    def depth_callback(self, img_msg):
        # Process depth image without CV Bridge
        self.depth = np.frombuffer(img_msg.data, dtype=np.float32).reshape(img_msg.height, img_msg.width)

        # Only publish a message for the image callback:
        return

        

    def publish_stopsign_message(self):
        if self.stopsign_present:
            center_x = int((self.left + self.right) / 2)
            center_y = int((self.top + self.bottom) / 2)
            center_depth = self.depth[center_y, center_x]
        else:
            center_depth = 1000.0 # Choose a very far away distance as a reasonable default in this case

        self.publisher.publish(DetectedObject(
            name="stop sign",
            isPresent=self.stopsign_present,
            depth=center_depth,
            top=self.top,
            left=self.left,
            bottom=self.bottom,
            right=self.right,
        ))

if __name__=="__main__":
    rospy.init_node("stop_sign_detector")
    detect = SignDetector()
    rospy.spin()
