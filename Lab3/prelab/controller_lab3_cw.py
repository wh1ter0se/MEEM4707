#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
	try:
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
		rospy.init_node('controller_lab3_cw')

		#@TODO : set the rate of publishing the command velocities
		rate = rospy.Rate(20) # 20 Hz
		count=0

		while not rospy.is_shutdown():

			v_cmd =0 # Command linear velocity
			omega_cmd = 0	# Command angular velocity     			

                        #TODO : determine v_cmd and omega_cmd for the desired 
                        ############## WRITE YOUR CODE BELOW ######################
			
			if count < (220*4): 
				#  220 counts (11 s) per side, four sides

				if (count % 220) < 100: 	
					# first 100 ticks (5 s) going straight at 0.2 m/s (2 m total)
					v_cmd = 0.2
					omega_cmd = 0

				elif (count % 220) < 200:					
					# next 100 ticks (5 s) turning cw at -90/5 deg/s (-90 deg total)
					v_cmd = 0
					omega_cmd = math.radians(-90/5)

				# spend 1 second buffer at rest
					
				count += 1

			
			################ YOUR CODE ENDS HERE ########################
			vel_msg=Twist()
			vel_msg.linear.x  = v_cmd
			vel_msg.angular.z = omega_cmd

			print("v_cmd : % 2f, omega_cmd : % 5.2f" %(v_cmd, omega_cmd))
			pub.publish(vel_msg)
			rate.sleep()

	except rospy.ROSInterruptException:
		pass
