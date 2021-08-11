#!/usr/bin/env python

from __future__ import print_function
import dvrk
import math
import sys
import rospy
import numpy
import PyKDL
import pandas as pd

data = pd.read_excel(r'~/CoppeliaSim_Edu_V4_0_0_Ubuntu18_04/DVRK_KINEMATIC_LOGGER_PSM1_PSM3/dvrk_kinematic_logger_test_2.xls')
df = pd.DataFrame(data, columns = ['PSM3_joint_1', 'PSM3_joint_2', 'PSM3_joint_3', 'PSM3_joint_4', 'PSM3_joint_5', 'PSM3_joint_6'])
data_array = df.to_numpy()

psm3_joint_values = data_array[:,0:6]

psm3 = dvrk.psm('PSM3')

for i in range(len(data_array)):
	
	goal_psm3 = numpy.copy(psm3_joint_values[i])

	psm3.move_joint(goal_psm3) 
	psm3.close_jaw()
print(rospy.get_caller_id(), ' <- joint direct complete')
