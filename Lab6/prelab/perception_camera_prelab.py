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
from sensor_msgs.msg import LaserScan
import roslib
import sys
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage

IS_SIMULATION = True


class recorder:

    scan_data = []
    image_data = []
    hsv_image = np.zeros([480, 640, 3]) * 255
    rgb_image = np.zeros([480, 640, 3]) * 255

    def scan_callback(self, sc_msg):
        recorder.scan_data = sc_msg.ranges

    def image_callback(self, img_msg):
        bridge = CvBridge()
        recorder.image_data = img_msg
        try:
            self.cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

    def compressed_image_callback(self, img_msg):
        # rgb_image =np.zeros([480,640,3])*255
        np_arr = np.fromstring(img_msg.data, np.uint8)
        recorder.rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def move(self):
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100)
        rospy.init_node("wall_follow")
        rate = rospy.Rate(10)  # 10hz
        scn_arr = recorder.scan_data

        while not rospy.is_shutdown():
            if not scn_arr:
                scn_arr = list(np.ones(360))
            else:
                scn_arr = recorder.scan_data

            if IS_SIMULATION:
                rgb_image = self.cv_image  # Comment out for real robot
            else:
                rgb_image = recorder.rgb_image  # Uncomment for real robot operation
                scn_arr = scn_arr[577:1153] + scn_arr[0:577]  # Uncomment for real robot
                scn_arr = [x if x > 0 else 1000 for x in scn_arr]  # Uncomment for real robot

            hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

            ####################@TODO write your code below##########################
            lower_range = np.array([0, 10, 10])  # HSV
            upper_range = np.array([10, 255, 255])  # HSV
            mask = cv2.inRange(hsv_image, lower_range, upper_range)  # Masking

            # print(hsv_image[0, 0])
            # print(np.unique(hsv_image.reshape(-1, 3), axis=0))

            print(mask)

            shape = np.shape(mask)  # calculate X and Y maximums, in px
            len_x = shape[1]  # height, *width, channels
            len_y = shape[0]  # *height, width, channels

            x_centerline = int(len_x / 2)  # centerline is in the middle
            x_centroid = 0
            num_passing = 0
            for x in range(len_x):  # calculate average passing x val
                for y in range(len_y):
                    if bool(mask[x, y]):
                        x_centroid += x
                        num_passing += 1
            x_centroid /= num_passing

            px_error = x_centerline - x_centroid
            px_tolerance = 16
            v_cmd = 0
            if abs(px_error) <= px_tolerance:  # within tolerance, stop moving
                w_cmd = 0
            elif px_error < 0:  # centroid is to the right
                w_cmd = math.radians(-360 / 15)  # rotate right at 4 RPM
            else:  # centroid is to the left
                w_cmd = math.radians(360 / 15)  # rotate left at 4 RPM

            cv2.imshow("mask", mask)
            cv2.waitKey(0)

            v_cmd = 0
            w_cmd = 0

            #########################################################################
            vel_msg = Twist()
            vel_msg.linear.x = v_cmd
            vel_msg.angular.z = w_cmd

            # print("ind:% f v_cmd : % 2f, omega_cmd : % 5.2f" %(ind, v_cmd, w_cmd))
            pub.publish(vel_msg)
            rate.sleep()


if __name__ == "__main__":

    try:
        rec = recorder()
        rospy.Subscriber("/scan", LaserScan, rec.scan_callback)
        rospy.Subscriber(
            "/camera/rgb/image_raw", Image, rec.image_callback
        )  # Comment out for real TB
        # rospy.Subscriber('/raspicam_node/image/compressed',CompressedImage,rec.compressed_image_callback) #Uncomment for real TB

        rec.move()
    except rospy.ROSInterruptException:
        pass
