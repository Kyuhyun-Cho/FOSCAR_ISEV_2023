#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
# from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

yaw = 0

arData = {"AX": 0.0, "AY": 0.0, "AZ": 0.0, "AW": 0.0}

def imuCB(msg):
    arData["AX"] = msg.orientation.x
    arData["AY"] = msg.orientation.y
    arData["AZ"] = msg.orientation.z
    arData["AW"] = msg.orientation.w

if __name__ == '__main__':
    rospy.init_node("get_yaw_from_imu", anonymous=True)

    rospy.Subscriber('/imu', Imu, imuCB)

    yawPub = rospy.Publisher("/yaw", Float64, queue_size = 1)

    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        (roll, pitch, yaw) = euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
        # print("BEFORE: ", yaw)
        yaw = math.degrees(yaw) 
        # print("AFTER: ", yaw)
        yawPub.publish(yaw)

        rate.sleep()