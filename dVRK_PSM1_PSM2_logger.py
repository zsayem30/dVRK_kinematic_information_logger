#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import cv2
import numpy as np
import xlsxwriter
import dvrk 
import sys
from scipy.spatial.transform import Rotation as R
import os
import arm_1_7
import camera

if sys.version_info.major < 3:
    input = raw_input

if __name__ == '__main__':

	rospy.init_node('topic_publisher')
	rate = rospy.Rate(140)

	PSM1 = arm_1_7.robot('PSM1')
	PSM3 = arm_1_7.robot('PSM3')

	input("		Press Enter to start logging...")

	workbook = xlsxwriter.Workbook('dvrk_kinematic_logger_test.xlsx')
	worksheet = workbook.add_worksheet()

	#start from the first cell
	#write the column headers for PSM1
	worksheet.write(0, 0, 'Time (Seconds)')
	worksheet.write(0, 1, 'Frame Number')
	worksheet.write(0, 2, 'PSM1_joint_1')
	worksheet.write(0, 3, 'PSM1_joint_2')
	worksheet.write(0, 4, 'PSM1_joint_3')
	worksheet.write(0, 5, 'PSM1_joint_4')
	worksheet.write(0, 6, 'PSM1_joint_5')
	worksheet.write(0, 7, 'PSM1_joint_6')
	worksheet.write(0, 8, 'PSM1_jaw_angle')
	worksheet.write(0, 9, 'PSM1_ee_x')
	worksheet.write(0, 10, 'PSM1_ee_y')
	worksheet.write(0, 11, 'PSM1_ee_z')

	worksheet.write(0, 12, 'PSM1_Orientation_Matrix_[1,1]')
	worksheet.write(0, 13, 'PSM1_Orientation_Matrix_[1,2]')
	worksheet.write(0, 14, 'PSM1_Orientation_Matrix_[1,3]')

	worksheet.write(0, 15, 'PSM1_Orientation_Matrix_[2,1]')
	worksheet.write(0, 16, 'PSM1_Orientation_Matrix_[2,2]')
	worksheet.write(0, 17, 'PSM1_Orientation_Matrix_[2,3]')

	worksheet.write(0, 18, 'PSM1_Orientation_Matrix_[3,1]')
	worksheet.write(0, 19, 'PSM1_Orientation_Matrix_[3,2]')
	worksheet.write(0, 20, 'PSM1_Orientation_Matrix_[3,3]')


	#write the column headers for PSM2
	worksheet.write(0, 21, 'PSM3_joint_1')
	worksheet.write(0, 22, 'PSM3_joint_2')
	worksheet.write(0, 23, 'PSM3_joint_3')
	worksheet.write(0, 24, 'PSM3_joint_4')
	worksheet.write(0, 25, 'PSM3_joint_5')
	worksheet.write(0, 26, 'PSM3_joint_6')
	worksheet.write(0, 27, 'PSM3_jaw_angle')

	worksheet.write(0, 28, 'PSM3_ee_x')
	worksheet.write(0, 29, 'PSM3_ee_y')
	worksheet.write(0, 30, 'PSM3_ee_z')

	worksheet.write(0, 31, 'PSM3_Orientation_Matrix_[1,1]')
	worksheet.write(0, 32, 'PSM3_Orientation_Matrix_[1,2]')
	worksheet.write(0, 33, 'PSM3_Orientation_Matrix_[1,3]')

	worksheet.write(0, 34, 'PSM3_Orientation_Matrix_[2,1]')
	worksheet.write(0, 35, 'PSM3_Orientation_Matrix_[2,2]')
	worksheet.write(0, 36, 'PSM3_Orientation_Matrix_[2,3]')

	worksheet.write(0, 37, 'PSM3_Orientation_Matrix_[3,1]')
	worksheet.write(0, 38, 'PSM3_Orientation_Matrix_[3,2]')
	worksheet.write(0, 39, 'PSM3_Orientation_Matrix_[3,3]')

	#write the column headers for ECM (4 joints)
	worksheet.write(0, 40, 'ECM_joint_1')
	worksheet.write(0, 41, 'ECM_joint_2')
	worksheet.write(0, 42, 'ECM_joint_3')
	worksheet.write(0, 43, 'ECM_joint_4')

	worksheet.write(0, 44, 'ECM_ee_x')
	worksheet.write(0, 45, 'ECM_ee_y')
	worksheet.write(0, 46, 'ECM_ee_z')

	worksheet.write(0, 47, 'ECM_Orientation_Matrix_[1,1]')
	worksheet.write(0, 48, 'ECM_Orientation_Matrix_[1,2]')
	worksheet.write(0, 49, 'ECM_Orientation_Matrix_[1,3]')

	worksheet.write(0, 50, 'ECM_Orientation_Matrix_[2,1]')
	worksheet.write(0, 51, 'ECM_Orientation_Matrix_[2,2]')
	worksheet.write(0, 52, 'ECM_Orientation_Matrix_[2,3]')

	worksheet.write(0, 53, 'ECM_Orientation_Matrix_[3,1]')
	worksheet.write(0, 54, 'ECM_Orientation_Matrix_[3,2]')
	worksheet.write(0, 55, 'ECM_Orientation_Matrix_[3,3]')
	#write the column headers for the cameras
	worksheet.write(0, 56, 'Left Camera Image')
	worksheet.write(0, 57, 'Right Camera Image')

	i = 0

	start_time = 0

	def callback_dummy(data):
		global i, start_time

		PSM1_jp = PSM1.get_current_joint_position()
		PSM1_jaw_angle = PSM1.get_current_jaw_position()
		
		if PSM1_jaw_angle[0] < 0:
			PSM1_jaw_angle[0] = 0
		PSM1_jaw_angle = [PSM1_jaw_angle[0]]

		PSM1_ee = PSM1.get_current_cartesian_position()
		PSM1_Orientation_Matrix = PSM1.get_current_orientation_matrix()

		PSM3_jp = PSM3.get_current_joint_position()
		PSM3_jaw_angle = PSM3.get_current_jaw_position()

		if PSM3_jaw_angle[0] < 0:
			PSM3_jaw_angle[0] = 0
		PSM3_jaw_angle = [PSM3_jaw_angle[0]]

		PSM3_ee = PSM3.get_current_cartesian_position()
		PSM3_Orientation_Matrix = PSM3.get_current_orientation_matrix()

		PSM1_data = np.concatenate((PSM1_jp, PSM1_jaw_angle, PSM1_ee, PSM1_Orientation_Matrix), axis = 0)
		PSM3_data = np.concatenate((PSM3_jp, PSM3_jaw_angle, PSM3_ee, PSM3_Orientation_Matrix), axis = 0)

		all_data = np.concatenate((PSM1_data, PSM3_data), axis = 0)

		time = data.header.stamp.secs + data.header.stamp.nsecs*10**(-9)

		if i != 0:
			Sequence = i
			time = time - start_time
			print(time)
			info_frame = [time, Sequence]

			all_data = np.concatenate((info_frame, all_data), axis = 0)

			for col in range(len(all_data)):
				worksheet.write(i, col, all_data[col])

		else:
			start_time = time
		i = i + 1

	rospy.Subscriber('/dvrk/PSM1/position_cartesian_current', PoseStamped, callback_dummy, queue_size = 1, buff_size = 1000000)

	try:
		rospy.spin()
	except rospy.ROSInterruptException as e:
		print("Error Running ROS." + e)
		pass

	workbook.close()