#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import csv

class recorder:

	scan_data = []

	def scan_callback(self,sc_msg):

		recorder.scan_data= sc_msg.ranges

	def move(self):
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
		rospy.init_node('wall_follow')
		rate = rospy.Rate(10)
		scn_arr = recorder.scan_data

		while not rospy.is_shutdown():
			rospy.Subscriber('/scan',LaserScan,rec.scan_callback)
			if not scn_arr:
				scn_arr = list(np.zeros(L))	# GAZEBO L=360 / REAL L = 1153
			else:
				scn_arr = recorder.scan_data
			L = len(scn_arr)
			res = float(360)/L

			#scn_arr=scn_arr[577:1153]+scn_arr[0:577] # Uncomment for real robot
			#scn_arr = [x if x>0 else 1000 for x in scn_arr] # Uncomment for real robot

			####################@TODO write your code below##########################

			#print(scn_arr) # "arr" is the scan data array(list of 360/1153 values for gazebo/real robot)
			d = min(scn_arr)
			ind = scn_arr.index(min(scn_arr))	# This is index of minimum distance value in scn_arr
			ang = ind*res			# This is angle of minimum distance value in scn_arr
			print(d, ang)

			v_cmd=0.2
			w_cmd=0.5


			#########################################################################
			vel_msg=Twist()
			vel_msg.linear.x = v_cmd
			vel_msg.angular.z = w_cmd

			#print("ind:% f v_cmd : % 2f, omega_cmd : % 5.2f" %(ind, v_cmd, w_cmd))
			pub.publish(vel_msg)
			rate.sleep()
	
if __name__ == '__main__':

	try:
		rec = recorder()
		rospy.Subscriber('/scan',LaserScan,rec.scan_callback)
		rec.move()
	except rospy.ROSInterruptException:
		pass
