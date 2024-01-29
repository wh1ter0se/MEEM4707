import rospy

rate=rospy.Rate(5)
Stop = False
count = 0
while ~Stop:
    count = count+1
    time = count/5
    if time < 2:
        print(count)
    else:
        Stop = True
