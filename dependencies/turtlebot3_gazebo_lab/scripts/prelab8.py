#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
import tf
import geometry_msgs
import roslib
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import csv
import os
from pidcont import *


class recorder:
  
	xs   = 0
	ys   = 0
	yaw  = 0
	vs   = 0
	ws   = 0
	v_c  = 0
	w_c  = 0
	path =np.zeros(5)
	scan_data = []
	def scan_callback(self,sc_msg):
		recorder.scan_data= sc_msg.ranges
		
	def odometrycallback(self, msg):
	
		recorder.xs = msg.pose.pose.position.x
		recorder.ys = msg.pose.pose.position.y
		recorder.vs = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
		recorder.ws = msg.twist.twist.angular.z
		quaternion = (
			msg.pose.pose.orientation.x,
			msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z,
			msg.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		#roll  = euler[0]
		#pitch = euler[1]
		recorder.yaw = euler[2]
		#print (recorder.yaw)

	
	def move(self):
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
		rospy.init_node('wall_follow')
		rate = rospy.Rate(10) # 10hz
		scn_arr = recorder.scan_data
		count = 0
		while not rospy.is_shutdown():
			rospy.Subscriber('/scan',LaserScan,rec.scan_callback)
			if not scn_arr:
				scn_arr = list(np.zeros(360))
			else:
				scn_arr = recorder.scan_data

			#Current estimated state of the robot
			#xc,yc,tc=est_pos
			xc = recorder.xs
			yc = recorder.ys
			tc = recorder.yaw
			rx = [0.75,0.75,1]
			ry = [0,0.5,0.5]
			ed=0
			edi=0
			for i in range(len(rx)):
				S=(rx[i]-xc)*np.cos(tc) + (ry[i]-yc)*np.sin(tc)
				d=(ry[i]-yc)*np.cos(tc) - (rx[i]-xc)*np.sin(tc)
				if S>0.08:
					break
			#print(S,d)
			v_cmd,w_cmd,ed,edi = pid_controller(d,ed,edi)
			
			dl=np.sqrt((rx[-1]-xc)**2 + (ry[-1]-yc)**2)
			if dl<0.05:
				v_cmd=0
				w_cmd=0

			#print(v_cmd, w_cmd)
		#########################################################################
	
			vel_msg=Twist()
			vel_msg.linear.x = v_cmd
			vel_msg.angular.z = w_cmd
			pub.publish(vel_msg)
			rate.sleep()
		
if __name__ == '__main__': 

	try:
		rec = recorder()
		rospy.Subscriber('/odom',Odometry,rec.odometrycallback)
		rospy.Subscriber('/scan',LaserScan,rec.scan_callback)
		rec.move()
	except rospy.ROSInterruptException:
		pass