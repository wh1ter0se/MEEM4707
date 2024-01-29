#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
	try:
		pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
		rospy.init_node('controller_lab3_cw')

		rate = rospy.Rate(20) # 20 Hz
		count=0

		a1 = 0.963668549396594
		a2 = -0.0428933081253249
		a3 = 0.00231657253017032
		a4 = -0.00592729823600611
		b = 0.287

		def V_command(V_target, omega_target):
			return (4.0*V_target*a1 - 4.0*a1*a3 + a2*a4*b - a2*b*omega_target)/(4.0*a1**2 - a2**2)
		
		def omega_command(V_target, omega_target):
			return (-4.0*V_target*a2 - 4.0*a1*a4*b + 4.0*a1*b*omega_target + 4.0*a2*a3)/(4.0*a1**2*b - a2**2*b)

		while not rospy.is_shutdown():

			v_cmd =0 # Command linear velocity
			omega_cmd = 0	# Command angular velocity     			
			
			if count < (400*4): 
				#  220 counts (11 s) per side, four sides

				if (count % 400) < 200: 	
					# first 200 ticks (10 s) going straight at 0.05 m/s (0.5 m total)
					v_target = 0.05
					omega_target = 0

				else:					
					# next 24 ticks (1.25 s) turning cw at -90/5 deg/s (-90 deg total)
					v_target = 0
					omega_target = math.radians(-90/10)

				v_cmd = V_command(v_target, omega_target)
				omega_cmd = omega_command(v_target, omega_target)
					
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
