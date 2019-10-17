#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import math
import rosbag
import rospy
import json
import csv

from std_msgs.msg import String
from scipy.stats import gamma, norm

pfile = open('/home/mehdi/tmp/params.csv', mode='w')
pwriter = csv.writer(pfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
pwriter.writerow([
	# 'sig_s_shape', 'sig_s_scale', 
	# 'sig_v_shape', 'sig_v_scale', 
	'sig_r_shape', 'sig_r_scale', 
	'sig_b_shape', 'sig_b_scale',
	# 'L_shape', 'L_scale', 
	# 'H_shape', 'H_scale', 
	# 'b_shape', 'b_scale', 
	# 'a_shape', 'a_scale',
	])







def callback(msg):
	params = json.loads(msg.data)

	row = [
		# params['sig_s']['shape'], params['sig_s']['scale'],
		# params['sig_v']['shape'], params['sig_v']['scale'],
		params['sig_r']['shape'], params['sig_r']['scale'],
		params['sig_b']['shape'], params['sig_b']['scale'],
		# params['L']['shape'], params['L']['scale'],
		# params['H']['shape'], params['H']['scale'],
		# params['b']['shape'], params['b']['scale'],
		# params['a']['shape'], params['a']['scale'],
	]

	pwriter.writerow(row)

	print row



def closing():
	pfile.close()


def listener():
    rospy.init_node('params_visualizer')
    rospy.Subscriber("/params", String, callback, queue_size=1)

    rospy.on_shutdown(closing)
    
    rospy.spin()

listener()

