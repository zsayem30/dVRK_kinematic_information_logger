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
    rate = rospy.Rate(10)
    
    psm3 = dvrk.psm("PSM3")

    left_cam = camera.camera('left')
    right_cam = camera.camera('right')

    orientation_Array = []
    Vector_Array = []
    input("     Press Enter to start logging...")

    for i in range(20):
        global orientation_Array, Vector_Array
        print(i)
        input("     Press Enter to save image...")
        image_left = left_cam.save_image()
        image_right = right_cam.save_image()
        psm3_orientation = np.copy(psm3.get_current_position().M)
        psm3_vector = np.copy(psm3.get_current_position().p)
        orientation_Array.append(psm3_orientation)
        Vector_Array.append(psm3_vector)

    try:
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print("Error Running ROS." + e)
        pass

    print("Printing orientation_Array")
    print(orientation_Array)
    print("Printing Vector_Array")
    print(Vector_Array)