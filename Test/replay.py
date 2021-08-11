#!/usr/bin/env python

from __future__ import print_function
import dvrk
import math
import sys
import rospy
import numpy
import PyKDL
import pandas as pd

data = pd.read_excel(r'~/CoppeliaSim_Edu_V4_0_0_Ubuntu18_04/DVRK_KINEMATIC_LOGGER_PSM1_PSM3/dvrk_kinematic_logger_test_1.xls')
df = pd.DataFrame(data, columns = ['PSM3_joint_1', 'PSM3_joint_2', 'PSM3_joint_3', 'PSM3_joint_4', 'PSM3_joint_5', 'PSM3_joint_6'])
data_array = df.to_numpy()

psm1_joint_values = data_array[:,0:6]
rate = 100

psm1 = dvrk.arm('PSM3')

psm1.home()

for i in range(len(data_array)):
	
	goal_psm1 = numpy.copy(psm1_joint_values[i])

	psm1.move_joint(goal_psm1, interpolate = False) 
	rospy.sleep(1.0 / rate)

print(rospy.get_caller_id(), ' <- joint direct complete')
