#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from enum import Enum

SIM = True

class recorder:

    scan_data = []
    image_data = []

    def scan_callback(self, sc_msg):
        recorder.scan_data = sc_msg.ranges

    def move(self):
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
        rospy.init_node("wall_follow")
        rate = rospy.Rate(10)  # 10hz
        count = 0
        arr = recorder.scan_data
        r = 3

        # added variables
        linear_speed = 0.1
        angular_kP = 0.045
        angular_tolerance = 5  # degrees
        follow_distance = 0.5  # m

        init_buffer = 10
        count = 0

        class EState(Enum):
            SCANNING = 1
            TARGETING = 2
            APPROACH = 3
            ALIGNING = 4
            FOLLOWING = 5
            COMPLETE = 6

        state = EState["SCANNING"]

        while not rospy.is_shutdown():
            rospy.Subscriber("/scan", LaserScan, rec.scan_callback)
            if not arr:
                arr = list(np.ones(360 if SIM else 1153))  # GAZEBO L=360 / REAL L = 1153
            else:
                arr = recorder.scan_data

            L = len(arr)
            res = float(360) / L

            # arr=arr[577:1153]+arr[0:577]			# Uncomment for real robot
            # arr = [x if x>0 else 1000 for x in arr]	# Uncomment for real robot
            # print(arr)

            ####################@TODO write your code below##########################

            
            arr_size = 400
            arr_sizem = 4
            resm = arr_sizem/arr_size 

            # a = [
            #     (arr_sizem/2) + resm * i for i in range(arr_size+1)
            # ]  # @TODO# example decide range of 'a'= [1 to 2) / SET your own range
            # b = [
            #     (arr_sizem/2) - resm * i for i in range(arr_size+1)
            # ]  # @TODO# example decide range of 'b'= [1 to -1) / SET your own range
            resm = arr_sizem / arr_size
            a=[(arr_sizem/2)-resm*i for i in range(arr_size)] #@TODO# example decide range of 'a'= [1 to 2) / SET your own range
            b=[(arr_sizem/2)-resm*i for i in range(arr_size)] #@TODO# example decide range of 'b'= [1 to -1) / SET your own range
            mat = np.zeros([len(a), len(b)])  # The HT vote matrix

            # Reading LIDAR scan points
            X = [
                val * np.cos(ind * 3.14 / 180 * res) for ind, val in enumerate(arr)
            ]  ## Xi array from laser scan points - the voters
            Y = [
                val * np.sin(ind * 3.14 / 180 * res) for ind, val in enumerate(arr)
            ]  ## Yi array from laser scan points - the voters

            for i in range(L):  # loop to pick the voters(laser scan points ) one by one
                if arr[i] < 5:  # @TODO# Condition to remove scan 'inf' values
                    for j in range(len(a)):  # inner loop to pick 'a' values one by one
                        if 0.25**2 - ((X[i] - a[j]) ** 2) >= 0:
                            b_temp1 = Y[i] - np.sqrt(
                                0.25**2 - ((X[i] - a[j]) ** 2)
                            )  # caculation for b
                            b_temp2 = Y[i] + np.sqrt(0.25**2 - ((X[i] - a[j]) ** 2))
                            ind_temp1 = np.floor(
                                len(b) * (b_temp1 - b[0]) / (b[-1] - b[0])
                            )  # determine the index for the 'b' value found above
                            ind_temp2 = np.floor(len(b) * (b_temp2 - b[0]) / (b[-1] - b[0]))
                            if b_temp1 is not np.nan and 0 <= ind_temp1 < len(
                                b
                            ):  # @TODO# condition to filter out unacceptable values/index of 'b'
                                mat[j][int(ind_temp1)] += 1
                            if b_temp2 is not np.nan and 0 <= ind_temp2 < len(b):
                                mat[j][int(ind_temp2)] += 1

            C = np.unravel_index(
                np.argmax(mat, axis=None), mat.shape
            )  # index of highest vote in HT matrix

            C_x = resm * C[0]
            C_y = resm * C[1]
            C_omega = math.degrees(math.atan(C_y / C_x)) if C_y is not None and C_x is not None else None
            # C_dist = math.sqrt(C_x**2 + C_y**2)
            print(C[0], C[1])
            #print(a[0],b[0],mat[0])

            d = min(arr)
            ind = arr.index(min(arr))  # This is index of minimum distance value in arr
            ang = ind * res  # This is angle of minimum distance value in arr
            # print(d, ang)

            if count < init_buffer:  # ignore bad initial lidar data
                w_cmd = 0
                v_cmd = 0
                count += 1

            elif state == EState["SCANNING"]:
                spin_seconds = 10
                w_cmd = math.radians(360 / 10)
                v_cmd = 0
                if ((count - init_buffer) / 10) > spin_seconds:
                    state = EState["TARGETING"]

            elif state == EState["TARGETING"]:
                print("[2] SCANNING (ang error = " + str(C_omega) + ")")

                w_cmd = math.radians(90 / 5)
                v_cmd = 0
                if (abs(C_omega) <= angular_tolerance) or (abs(360 - C_omega) <= angular_tolerance):
                    w_cmd = 0
                    v_cmd = 0
                    state = EState["APPROACH"]

            elif state == EState["APPROACH"]:
                print("[3] APPROACH (dist error = " + str(d) + ")")
                w_cmd = 0
                v_cmd = linear_speed
                if d <= follow_distance:
                    v_cmd = 0
                    w_cmd = 0
                    state = EState["ALIGNING"]

            elif state == EState["ALIGNING"]:
                print("[4] ALIGNING (ang error = " + str(abs(90 - C_omega)) + ")")
                w_cmd = math.radians(-90 / 5)
                v_cmd = 0
                if abs(90 - C_omega) <= angular_tolerance:
                    state = EState["FOLLOWING"]

            elif state == EState["FOLLOWING"]:
                angle_error = min([90 - C_omega, (90) + (360 - C_omega)])
                print(
                    "[5]] FOLLOWING (angle error = "
                    + str(angle_error)
                    + ", dist error = "
                    + str(d)
                    + ")"
                )
                w_cmd = -angle_error * angular_kP  # P-controller on angle
                v_cmd = linear_speed

            # v_cmd = 0  # @TODO
            # w_cmd = 0  # @TODO
            # print(v_cmd, w_cmd)

            #########################################################################
            vel_msg = Twist()
            vel_msg.linear.x = v_cmd*0
            vel_msg.angular.z = w_cmd*0

            # print("ind% f v_cmd : % 2f, omega_cmd : % 5.2f" %(ind, v_cmd, w_cmd))
            pub.publish(vel_msg)
            rate.sleep()


if __name__ == "__main__":
    try:
        rec = recorder()
        rospy.Subscriber("/scan", LaserScan, rec.scan_callback)
        rec.move()
    except rospy.ROSInterruptException:
        pass
