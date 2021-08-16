#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
import cv2
import numpy as np
import xlsxwriter
import dvrk 
import sys
from scipy.spatial.transform import Rotation as R
import os
import camera

if sys.version_info.major < 3:
    input = raw_input

if __name__ == '__main__':

    rospy.init_node('topic_publisher')
    rate = rospy.Rate(140)

    left_cam = camera.camera('left')
    right_cam = camera.camera('right')

    input("     Press Enter to start logging...")

    def callback_dummy(data):

        image_left = left_cam.save_image()
        image_right = right_cam.save_image()


    rospy.Subscriber('/dVRK/left/decklink/camera/camera_info', CameraInfo, callback_dummy, queue_size = 1, buff_size = 1000000)

    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print("Error Running ROS." + e)
        pass
