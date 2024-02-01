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
from enum import Enum

class recorder:

	scan_data = []

	def scan_callback(self,sc_msg):

		recorder.scan_data= sc_msg.ranges

	def move(self):
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
		rospy.init_node('wall_follow')
		rate = rospy.Rate(10)
		scn_arr = recorder.scan_data


		linear_speed = 0.1
		angular_kP = 0.2
		angular_tolerance = 5 # degrees

		follow_distance = 0.2 # m
		follow_distance_tolerance = 0.05 # m
		heading = 0
		distance = 0

		state = EState['ALIGNING']
		class EState(Enum):
			POSITIONING = 1
			ALIGNING = 2
			FOLLOWING = 3
			COMPLETE = 4

		while not rospy.is_shutdown():
			rospy.Subscriber('/scan',LaserScan,rec.scan_callback)
			if not scn_arr:
				scn_arr = list(np.zeros(360))	# GAZEBO L=360 / REAL L = 1153
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

			if state == EState['POSITIONING']:
				w_cmd = ang*angular_kP # P-controller on angle
				v_cmd = linear_speed
				if d < follow_distance:
					v_cmd = 0
					w_cmd = 0
					state = EState['ALIGNING']

			elif state == EState['ALIGNING']:
				w_cmd = (ang-90)*angular_kP # P-controller on angle
				v_cmd = 0
				if (ang-90) < angular_tolerance:
					state = EState['ALIGNING']

			elif state == EState['FOLLOWING']:
				if abs(d-follow_distance) > follow_distance_tolerance:
					ang += (d-follow_distance)*angular_kP
				w_cmd = (ang)*angular_kP # P-controller on angle
				v_cmd = follow_distance

				




			# v_cmd=0.2
			# w_cmd=0.5

			# 


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
