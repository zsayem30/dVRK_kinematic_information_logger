#!/usr/bin/env python

from __future__ import print_function
import dvrk
import math
import sys
import rospy
import numpy
import PyKDL
import pandas as pd

if sys.version_info.major < 3:
    input = raw_input

psm1 = dvrk.psm('PSM1')
psm2 = dvrk.psm('PSM2')

psm1.home()
psm2.home()


psm1.move_jaw(0.5235987756) 

input("		Press Enter to start logging...")

position = psm1.get_current_jaw_position()


print(rospy.get_caller_id(), ' <- joint direct complete')
