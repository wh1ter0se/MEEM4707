#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from tf2_msgs.msg import TFMessage
import roslib
import sys
from std_msgs.msg import String


class recorder:

	pos=[]

	def tf_callback(self,tf_msg):
		#recorder.link=tf_msg.transforms
		if(tf_msg.transforms[0].child_frame_id=='base_footprint'):
			#print(tf_msg.transforms[0].transform.translation.x,tf_msg.transforms[0].transform.translation.y)
			recorder.pos=[tf_msg.transforms[0].transform.translation.x,tf_msg.transforms[0].transform.translation.y]
	def move(self):

		rospy.init_node('map_reader')
		rate = rospy.Rate(1) # 1hz
		map_arr = recorder.pos
		while not rospy.is_shutdown():
			if not map_arr:
				map_arr = list(np.zeros(2))
			else:
				map_arr = recorder.pos

			####################@TODO write your code below##########################
			print(map_arr)
			#########################################################################
			rate.sleep()
	
if __name__ == '__main__':

	try:
		rec = recorder()
		rospy.Subscriber('/tf',TFMessage,rec.tf_callback)
		rec.move()
	except rospy.ROSInterruptException:
		pass
