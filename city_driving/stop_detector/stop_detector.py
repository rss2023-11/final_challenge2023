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

        scan_topic = rospy.get_param("~stopsign_topic", "/stopsign")
        self.publisher = rospy.Publisher(scan_topic, DetectedObject, queue_size=20)
        self.subscriber = rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.callback)

    def callback(self, img_msg):
        # Process image without CV Bridge
        np_img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        bgr_img = np_img[:,:,:3]
        #print(bgr_img)
        #print(bgr_img.shape)
        rgb_img = bgr_img[:,:,::-1]
        #rgb_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)

        is_stop_sign, bounding_box = self.detector.predict(rgb_img)
        left, top, right, bottom = bounding_box

        self.publisher.publish(DetectedObject(
            name="stop sign",
            top=top,
            left=left,
            bottom=bottom,
            right=right,
            isPresent=is_stop_sign,
        ))

if __name__=="__main__":
    rospy.init_node("stop_sign_detector")
    detect = SignDetector()
    rospy.spin()
