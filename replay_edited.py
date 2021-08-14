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
df = pd.DataFrame(data)

data_array = df.to_numpy()
df_header = list(df.columns)

time = data_array[:,1]

psm1_joint_values = data_array[:, 3:9]
psm1_jaw = data_array[:, 9]
psm3_joint_values = data_array[:, 42:48]
psm3_jaw = data_array[:, 48]

ecm_joint_values = data_array[:, 81:85]

print(psm1_jaw[0])
print(psm3_jaw[0])

delay = [t - s for s, t in zip(time, time[1:])]



psm1 = dvrk.arm('PSM1')
psm3 = dvrk.arm('PSM3')
ecm = dvrk.arm('ECM')

psm1.home()
psm3.home()
ecm.home()

psm1_replayed_joint_values = np.empty([len(psm1_joint_values), len(psm1_joint_values[0,:])])
psm1_replayed_jaw_values = np.empty()
psm3_replayed_joint_values = np.empty([len(psm3_joint_values), len(psm3_joint_values[0,:])])
ecm_replayed_joint_values = np.empty([len(ecm_joint_values), len(ecm_joint_values[0,:])])

goal_start_psm1 = np.copy(psm1_joint_values[0])
goal_start_psm3 = np.copy(psm3_joint_values[0])
goal_start_ecm = np.copy(ecm_joint_values[0])

print('Moving arms to initial starting position...')
psm1.move_joint(goal_psm1, interpolate = False)
psm3.move_joint(goal_psm3, interpolate = False)
ecm.move_joint(goal_ecm, interpolate = False)

rospy.sleep(2)

psm1_replayed_joint_values[0, :] = psm1.get_current_joint_position()
psm3_replayed_joint_values[0, :] = psm3.get_current_joint_position()
ecm_replayed_joint_values[0, :] = ecm.get_current_joint_position()


print('Arm at starting position...Motion about to start...')

for i in range(len(delay)):
	
	goal_psm1 = np.copy(psm1_joint_values[i + 1])
	goal_psm3 = np.copy(psm3_joint_values[i + 1])
	goal_ecm = np.copy(ecm_joint_values[i + 1])

	goal_psm1_jaw = np.copy(PSM1_jaw[i + 1])
	goal_psm3_jaw = np.copy(PSM3_jaw[i + 1])

	psm1.move_joint(goal_psm1, interpolate = False)
	psm1.move_jaw(goal_psm1_jaw, interpolate = False)
	
	psm3.move_joint(goal_psm3, interpolate = False)
	psm3.move_jaw(goal_psm3_jaw, interpolate = False)
	
	ecm.move_joint(goal_ecm, interpolate = False)

	rospy.sleep(delay[i])

	psm1_replayed_joint_values[i + 1, :] = psm1.get_current_joint_position()
	psm1_replayed_jaw_values[i + 1, :] = psm1.get_current_jaw_position()
	
	psm3_replayed_joint_values[i + 1, :] = psm3.get_current_joint_position()
	psm3_replayed_jaw_values[i + 1, :] = psm3.get_current_jaw_position()
	
	ecm_replayed_joint_values[i + 1, :] = ecm.get_current_joint_position()

psm1_replayed_data = np.concatenate((psm1_replayed_joint_values, psm1_replayed_jaw_values), axis = 0)
psm3_replayed_data = np.concatenate((psm3_replayed_joint_values, psm3_replayed_jaw_values), axis = 0)

all_data = np.concatenate((psm1_replayed_data, psm3_replayed_data, ecm_replayed_joint_values), axis = 0)

replayed_joint_values_Df = pd.DataFrame(all_data, columns = df_header)

filepath = 'replayed_data.xlsx'

psm1_replayed_joint_values_Df.to_excel(filepath, index = False)

print(rospy.get_caller_id(), ' <- joint direct complete')

