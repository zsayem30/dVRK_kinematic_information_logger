#!/usr/bin/env python

from __future__ import print_function
import dvrk
import math
import sys
import rospy
import numpy as np
import PyKDL
import pandas as pd
from numpy import linalg as LA

data = pd.read_excel(r'dvrk_kinematic_logger_test.xls')
df = pd.DataFrame(data, columns = ['PSM3_joint_1', 'PSM3_joint_2', 'PSM3_joint_3', 'PSM3_joint_4', 'PSM3_joint_5', 'PSM3_joint_6'])

data_array = df.to_numpy()
df_header = list(df.columns)

psm1_joint_values = data_array[:,0:6]
rate = 110

psm1 = dvrk.arm('PSM3')

psm1.home()

psm1_replayed_joint_values = np.empty([len(psm1_joint_values), len(psm1_joint_values[0,:])])

for i in range(len(data_array)):
	
	goal_psm1 = np.copy(psm1_joint_values[i])
	psm1.move_joint(goal_psm1, interpolate = False)
	rospy.sleep(1.0 / rate)
	psm1_replayed_joint_values[i,:] = psm1.get_current_joint_position()
	print(psm1_replayed_joint_values[i,:])

psm1_replayed_joint_values_Df = pd.DataFrame(psm1_replayed_joint_values, columns = df_header)
filepath = 'replayed_data.xlsx'
psm1_replayed_joint_values_Df.to_excel(filepath, index = False)

print(rospy.get_caller_id(), ' <- joint direct complete')
