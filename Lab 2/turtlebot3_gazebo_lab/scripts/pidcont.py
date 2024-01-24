#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import numpy as np

def pid_controller(d,ed,edi):

	kp= #To be set by students
	ki= #To be set by students
	kd= #To be set by students

	
	edi=edi+ed #error integration

	w=max(min(1.86, kp*d + ki*edi + kd*(d-ed)),-1.86)
	v=0.15*np.exp(-100*abs(w)) # Max speed set as 0.15
	
	ed=d #previous error

	print(d,w,v)
	return v,w,ed,edi
	#######################################################################################

