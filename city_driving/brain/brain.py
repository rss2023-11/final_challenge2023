#!/usr/bin/env python2

import rospy

from final_challenge2023.msg import DetectedObject
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class Brain:
    DRIVE_TOPIC = rospy.get_param("~drive_topic")

    STOPSIGN_TOPIC = rospy.get_param("~stopsign_topic", "/stopsign")
    LINE_FOLLOWING_TOPIC = rospy.get_param("~line_following_drive_topic", "/line_following_drive")

    def __init__(self):
        self.line_following_drive = None
        self.stopsign_present = False
        self.stopsign_distance = None

        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10) # Publish the drive actions to the robot

        self.stopsign_sub = rospy.Subscriber(self.STOPSIGN_TOPIC, DetectedObject, self.stopsign_callback)
        self.line_following_sub = rospy.Subscriber(self.LINE_FOLLOWING_TOPIC, AckermannDriveStamped, self.line_following_callback)

        # Main loop--publish a drive command for the robot 100 times per second
        while not rospy.is_shutdown():
            self.publish_best_drive()
            rospy.sleep(0.01) # Publish at a maximuim rate of 100 Hz

    def stopsign_callback(self, msg):
        if not msg.isPresent: # If there isn't a stop sign, update accordingly
            self.stopsign_present = False
            return
        
        depth = msg.depth
        # If the depth is super far, then there isn't a stop sign nearby
        if depth > 2.0:
            self.stopsign_present = False
            return
        
        # Depth detection is noisy below about 40cm--ignore super close results
        if depth < 0.35:
            return
        
        # TODO: Maybe check that the size for the stop sign is reasonable?
        
        self.stopsign_present = True
        self.stopsign_distance = depth

    def line_following_callback(self, msg):
        self.line_following_drive = msg

    def publish_best_drive(self):
        # Right now it just uses the line follower's drive messge:
        if self.line_following_drive is not None:
            self.drive_pub.publish(self.line_following_drive)

if __name__=="__main__":
    rospy.init_node("brain")
    detect = Brain()
    rospy.spin()
