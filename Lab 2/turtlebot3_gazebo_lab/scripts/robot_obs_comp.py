#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__': 
        
        try:         
	 pub= rospy.Publisher('robot1/cmd_vel',Twist,queue_size=100)
	 rospy.init_node('robot_obs_comp')
	 
	 rate = rospy.Rate(20) # example:20Hz
 
         while not rospy.is_shutdown():

		v_cmd = 0.05
		omega_cmd = 0.1
		print('Dynamic Obstacle in the path!!')
		vel_msg = Twist()
		vel_msg.linear.x = min(max(-0.25,v_cmd),0.25)
		vel_msg.angular.z = min(max(-1.8,omega_cmd),1.8)

		pub.publish(vel_msg)
		rate.sleep()

        except rospy.ROSInterruptException:
          pass





    
