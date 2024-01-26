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
from path_planning import *
from stage_1 import *

#from stage2 import *
#from stage3 import *

class recorder:

	scan_data = []
	map_data = []
	image_data = np.zeros([640,480])
	pos=[0,0,0]

	def map_callback(self,map_msg):
		recorder.map_data= map_msg.data
	
	def tf_callback(self,tf_msg):
		#recorder.link=tf_msg.transforms
		if(tf_msg.transforms[0].child_frame_id=='base_footprint'):
			x_temp = tf_msg.transforms[0].transform.translation.x
			y_temp = tf_msg.transforms[0].transform.translation.y
			quaternion = (tf_msg.transforms[0].transform.rotation.x,
				 tf_msg.transforms[0].transform.rotation.y,
				 tf_msg.transforms[0].transform.rotation.z,
				 tf_msg.transforms[0].transform.rotation.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)
			roll = euler[0]
			pitch = euler[1]
			yaw = euler[2]
			recorder.pos=[x_temp,y_temp,yaw]

	def scan_callback(self,sc_msg):
		recorder.scan_data= sc_msg.ranges

	"""def image_callback(self,img_msg):
		bridge = CvBridge()
		#recorder.image_data= img_msg
		try:
			recorder.image_data = bridge.imgmsg_to_cv2(img_msg, "bgr8")
		except CvBridgeError as e:
		print(e)"""

	def move(self):
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
		rospy.init_node('comp')
		rate = rospy.Rate(5) # 5hz
		map_arr  = recorder.map_data
		scan_arr = recorder.scan_data
		count=0
		xi=0
		yi=0
		thi=0
		flag=True
		ed=0
		edi=0
		while not rospy.is_shutdown():
			if not scan_arr:
				scan_arr = list(np.ones(L))	# GAZEBO L=360 / REAL L = 1153
			else:
				scan_arr = recorder.scan_data
				#scan_arr=[x if x>0 else 10000 for x in scan_arr] # uncomment for REAL ROBOT

			L = len(scan_arr)
			resL = float(360)/L
			#scan_arr=scan_arr[577:1153]+scan_arr[0:577]			# Uncomment for real robot

			#Reading LIDAR scan points FOR GAZEBO
			X_scan=[val*np.cos(ind*3.14/180*resL) for ind,val in enumerate (scan_arr) if val<2.5] #x cordinates of laser scanned point with in 2.5 m
			Y_scan=[val*np.sin(ind*3.14/180*resL) for ind,val in enumerate (scan_arr) if val<2.5] #y cordinates of laser scanned point with in 2.5 m

			#Reading estimated positon and orientation, [x,y,theta]
			est_pos= recorder.pos # estimated position of the robot
			#print(est_pos)
			if count<5:
				xi=est_pos[0]
				yi=est_pos[1]
				thi=est_pos[2]
				count=count+1
				est_pos[0]=(est_pos[0]-xi)*np.cos(thi) + (est_pos[1]-yi)*np.sin(thi)
				est_pos[1]=(est_pos[1]-yi)*np.cos(thi) - (est_pos[0]-xi)*np.sin(thi)
				est_pos[2]=est_pos[2]-thi

				#print(est_pos)

			#Reading Map
			map_arr=list(map_arr)
			X_map=[-10+0.05*(ind%384) for ind,val in enumerate (map_arr) if val>0] #x cordinates of wall points and cylinder obstacle points
			Y_map=[-10+0.05*(ind//384) for ind,val in enumerate (map_arr) if val>0] #y cordinates of wall points and cylinder obstacle points

			#plt.scatter(X_map,Y_map) # uncomment to plot the points
			#plt.show() # uncomment to plot the points

			############################write your code here########################
	
			if flag:
				flag=False
				rx,ry=path(est_pos,X_map,Y_map) # compute x,y cordinates of path in meters
			v_cmd,w_cmd,ind,ed,edi= stage1(est_pos,rx,ry,ind,ed,edi) # You can change the inputs or function as you wish

			#print(v_cmd,w_cmd)


			### THIS IS EXAMPLE HOW TO divide each stage code ###
			"""if(False):                                                #(put a suitable condition to implement stage 1):
				v_cmd,w_cmd = stage1(est_pos,X_map,Y_map,X_scan,Y_scan) # You can change the inputs or function as you wish

			elif(True):                                               #(put a suitable condition to implement stage 2):
				v_cmd,w_cmd = stage2(est_pos,X_map,Y_map,X_scan,Y_scan) # You can change the inputs or function as you wish 

			else:
				print("Currently in Stage 3")
				v_cmd,w_cmd = stage3(est_pos,X_map,Y_map,mask)          # You can change the inputs or function as you wish

			print(v_cmd,w_cmd)"""

			#########################################################################
	
			vel_msg=Twist()
			vel_msg.linear.x = v_cmd
			vel_msg.angular.z = w_cmd
			pub.publish(vel_msg)
			rate.sleep()
		
if __name__ == '__main__':

	try:
		rec = recorder()
		rospy.Subscriber('/map',OccupancyGrid,rec.map_callback)
		rospy.Subscriber('/tf', TFMessage,rec.tf_callback)
		rospy.Subscriber('/scan',LaserScan,rec.scan_callback)
		# rospy.Subscriber('/camera/rgb/image_raw',Image,rec.image_callback)
		rec.move()
	except rospy.ROSInterruptException:
		pass