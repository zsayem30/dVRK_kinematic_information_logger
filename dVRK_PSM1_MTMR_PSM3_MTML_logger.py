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
import mtm

if sys.version_info.major < 3:
    input = raw_input

if __name__ == '__main__':

	rospy.init_node('topic_publisher')
	rate = rospy.Rate(140)

	PSM1 = arm_1_7.robot('PSM1')
	PSM3 = arm_1_7.robot('PSM3')
	MTML = mtm.robot('MTML')
	MTMR = mtm.robot('MTMR')

	input("		Press Enter to start logging...")

	workbook = xlsxwriter.Workbook('dvrk_kinematic_logger_test_PSM_MTM.xlsx')
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


	#write the column headers for MTMR
	worksheet.write(0, 21, 'MTMR_joint_1')
	worksheet.write(0, 22, 'MTMR_joint_2')
	worksheet.write(0, 23, 'MTMR_joint_3')
	worksheet.write(0, 24, 'MTMR_joint_4')
	worksheet.write(0, 25, 'MTMR_joint_5')
	worksheet.write(0, 26, 'MTMR_joint_6')
	worksheet.write(0, 27, 'MTMR_joint_7')

	worksheet.write(0, 28, 'MTMR_jaw_angle')

	worksheet.write(0, 29, 'MTMR_ee_x')
	worksheet.write(0, 30, 'MTMR_ee_y')
	worksheet.write(0, 31, 'MTMR_ee_z')

	worksheet.write(0, 32, 'MTMR_Orientation_Matrix_[1,1]')
	worksheet.write(0, 33, 'MTMR_Orientation_Matrix_[1,2]')
	worksheet.write(0, 34, 'MTMR_Orientation_Matrix_[1,3]')

	worksheet.write(0, 35, 'MTMR_Orientation_Matrix_[2,1]')
	worksheet.write(0, 36, 'MTMR_Orientation_Matrix_[2,2]')
	worksheet.write(0, 37, 'MTMR_Orientation_Matrix_[2,3]')

	worksheet.write(0, 38, 'MTMR_Orientation_Matrix_[3,1]')
	worksheet.write(0, 39, 'MTMR_Orientation_Matrix_[3,2]')
	worksheet.write(0, 40, 'MTMR_Orientation_Matrix_[3,3]')

	#write the column headers for PSM3
	worksheet.write(0, 41, 'PSM3_joint_1')
	worksheet.write(0, 42, 'PSM3_joint_2')
	worksheet.write(0, 43, 'PSM3_joint_3')
	worksheet.write(0, 44, 'PSM3_joint_4')
	worksheet.write(0, 45, 'PSM3_joint_5')
	worksheet.write(0, 46, 'PSM3_joint_6')
	worksheet.write(0, 47, 'PSM3_jaw_angle')
	worksheet.write(0, 48, 'PSM3_ee_x')
	worksheet.write(0, 49, 'PSM3_ee_y')
	worksheet.write(0, 50, 'PSM3_ee_z')

	worksheet.write(0, 51, 'PSM3_Orientation_Matrix_[1,1]')
	worksheet.write(0, 52, 'PSM3_Orientation_Matrix_[1,2]')
	worksheet.write(0, 53, 'PSM3_Orientation_Matrix_[1,3]')

	worksheet.write(0, 54, 'PSM3_Orientation_Matrix_[2,1]')
	worksheet.write(0, 55, 'PSM3_Orientation_Matrix_[2,2]')
	worksheet.write(0, 56, 'PSM3_Orientation_Matrix_[2,3]')

	worksheet.write(0, 57, 'PSM3_Orientation_Matrix_[3,1]')
	worksheet.write(0, 58, 'PSM3_Orientation_Matrix_[3,2]')
	worksheet.write(0, 59, 'PSM3_Orientation_Matrix_[3,3]')


	#write the column headers for MTML
	worksheet.write(0, 60, 'MTML_joint_1')
	worksheet.write(0, 61, 'MTML_joint_2')
	worksheet.write(0, 62, 'MTML_joint_3')
	worksheet.write(0, 63, 'MTML_joint_4')
	worksheet.write(0, 64, 'MTML_joint_5')
	worksheet.write(0, 65, 'MTML_joint_6')
	worksheet.write(0, 66, 'MTML_joint_7')

	worksheet.write(0, 67, 'MTML_jaw_angle')

	worksheet.write(0, 68, 'MTML_ee_x')
	worksheet.write(0, 69, 'MTML_ee_y')
	worksheet.write(0, 70, 'MTML_ee_z')

	worksheet.write(0, 71, 'MTML_Orientation_Matrix_[1,1]')
	worksheet.write(0, 72, 'MTML_Orientation_Matrix_[1,2]')
	worksheet.write(0, 73, 'MTML_Orientation_Matrix_[1,3]')

	worksheet.write(0, 74, 'MTML_Orientation_Matrix_[2,1]')
	worksheet.write(0, 75, 'MTML_Orientation_Matrix_[2,2]')
	worksheet.write(0, 76, 'MTML_Orientation_Matrix_[2,3]')

	worksheet.write(0, 77, 'MTML_Orientation_Matrix_[3,1]')
	worksheet.write(0, 78, 'MTML_Orientation_Matrix_[3,2]')
	worksheet.write(0, 79, 'MTML_Orientation_Matrix_[3,3]')



	#write the column headers for ECM (4 joints)
	worksheet.write(0, 80, 'ECM_joint_1')
	worksheet.write(0, 81, 'ECM_joint_2')
	worksheet.write(0, 82, 'ECM_joint_3')
	worksheet.write(0, 83, 'ECM_joint_4')

	worksheet.write(0, 84, 'ECM_ee_x')
	worksheet.write(0, 85, 'ECM_ee_y')
	worksheet.write(0, 86, 'ECM_ee_z')

	worksheet.write(0, 87, 'ECM_Orientation_Matrix_[1,1]')
	worksheet.write(0, 88, 'ECM_Orientation_Matrix_[1,2]')
	worksheet.write(0, 89, 'ECM_Orientation_Matrix_[1,3]')

	worksheet.write(0, 90, 'ECM_Orientation_Matrix_[2,1]')
	worksheet.write(0, 91, 'ECM_Orientation_Matrix_[2,2]')
	worksheet.write(0, 92, 'ECM_Orientation_Matrix_[2,3]')

	worksheet.write(0, 93, 'ECM_Orientation_Matrix_[3,1]')
	worksheet.write(0, 94, 'ECM_Orientation_Matrix_[3,2]')
	worksheet.write(0, 95, 'ECM_Orientation_Matrix_[3,3]')
	#write the column headers for the cameras
	worksheet.write(0, 96, 'Left Camera Image')
	worksheet.write(0, 97, 'Right Camera Image')

	i = 0

	start_time = 0

	def callback_dummy(data):
		global i, start_time

		#Get PSM1 data
		PSM1_jp = PSM1.get_current_joint_position()
		PSM1_jaw_angle = PSM1.get_current_jaw_position()
		
		if PSM1_jaw_angle[0] < 0:
			PSM1_jaw_angle[0] = 0
		PSM1_jaw_angle = [PSM1_jaw_angle[0]]

		PSM1_ee = PSM1.get_current_cartesian_position()
		PSM1_Orientation_Matrix = PSM1.get_current_orientation_matrix()
		PSM1_data = np.concatenate((PSM1_jp, PSM1_jaw_angle, PSM1_ee, PSM1_Orientation_Matrix), axis = 0)
		#Get MTMR data
		MTMR_jp = MTMR.get_current_joint_position()
		MTMR_jaw_angle = MTMR.get_current_jaw_position()

		if MTMR_jaw_angle[0] < 0:
			MTMR_jaw_angle[0] = 0
		MTMR_jaw_angle = [MTMR_jaw_angle[0]]

		MTMR_ee = MTMR.get_current_cartesian_position()
		MTMR_Orientation_Matrix = MTMR.get_current_orientation_matrix()
		MTMR_data = np.concatenate((MTMR_jp, MTMR_jaw_angle, MTMR_ee, MTMR_Orientation_Matrix), axis = 0)
		PSM3_jp = PSM3.get_current_joint_position()
		PSM3_jaw_angle = PSM3.get_current_jaw_position()

		if PSM3_jaw_angle[0] < 0:
			PSM3_jaw_angle[0] = 0
		PSM3_jaw_angle = [PSM3_jaw_angle[0]]

		PSM3_ee = PSM3.get_current_cartesian_position()
		PSM3_Orientation_Matrix = PSM3.get_current_orientation_matrix()
		PSM3_data = np.concatenate((PSM3_jp, PSM3_jaw_angle, PSM3_ee, PSM3_Orientation_Matrix), axis = 0)
		all_data = np.concatenate((PSM1_data, PSM3_data), axis = 0)

		MTML_jp = MTML.get_current_joint_position()
		MTML_jaw_angle = MTML.get_current_jaw_position()

		if MTML_jaw_angle[0] < 0:
			MTML_jaw_angle[0] = 0
		MTML_jaw_angle = [MTML_jaw_angle[0]]

		MTML_ee = MTML.get_current_cartesian_position()
		MTML_Orientation_Matrix = MTML.get_current_orientation_matrix()
		MTML_data = np.concatenate((MTML_jp, MTML_jaw_angle, MTML_ee, MTML_Orientation_Matrix), axis = 0)

		all_data = np.concatenate((PSM1_data, MTMR_data, PSM3_data, MTML_data), axis = 0)
		
		time = data.header.stamp.secs + data.header.stamp.nsecs*10**(-9)

		if i != 0:
			Sequence = i
			time = time - start_time
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