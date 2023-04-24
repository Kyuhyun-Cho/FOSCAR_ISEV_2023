#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import sys,os
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj, transform
from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, GPSMessage, CtrlCmd, EventInfo
from morai_msgs.srv import MoraiEventCmdSrv
from sensor_msgs.msg import PointCloud2, CompressedImage
from calib import LIDAR2CAMTransform


class Fusion():
    def __init__(self):
        rospy.init_node('sensor_fusion', anonymous=True)

        self.lidar2cam = LIDAR2CAMTransform()

        self.lidar = None
        self.cam = None

        # # Publisher
    
        # Subscriber
        rospy.Subscriber("/velodyne_points", PointCloud2, self.lidarCB)
        rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.imageCB)

    
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.lidar2cam(self.lidar, self.cam)

            rate.sleep()

    
    rospy.spin()

    def lidarCB(self, data) :
        self.lidar = data

    def imageCB(self, data) :
        self.cam = self.bridge.compressed_imgmsg_to_cv2(data)




if __name__ == '__main__':
    try:
        fusion= Fusion()
    except rospy.ROSInterruptException:
        pass