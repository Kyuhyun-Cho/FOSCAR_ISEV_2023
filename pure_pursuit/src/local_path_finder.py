#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import sys,os
import rospy
import tf
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point,PoseStamped
from pyproj import Proj, transform
from morai_msgs.msg import GPSMessage,CtrlCmd
from sensor_msgs.msg import Imu
from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int64,Float64
import time
import numpy as np
# from math import cos, sin
from math import *

import rospkg

class pathReader :
    def __init__(self, pkg_name):
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path(pkg_name)

    def read_txt(self,file_name):
        full_file_name = self.file_path + "/path/" + file_name
        openFile = open(full_file_name, 'r')
        global_path=Path()
        
        global_path.header.frame_id='map'
        line = openFile.readlines()
        for i in line :
            tmp = i.split()
            read_pose = PoseStamped()
            read_pose.pose.position.y = float(tmp[1])
            read_pose.pose.position.x = float(tmp[0])
            read_pose.pose.position.z = float(tmp[2])
            read_pose.pose.orientation.x = 0
            read_pose.pose.orientation.y = 0
            read_pose.pose.orientation.z = 0
            read_pose.pose.orientation.w = 1
            global_path.poses.append(read_pose)
        
        openFile.close()
        return global_path
    

    def find_local_path(global_path, status_msg):
        local_path=Path()
        current_x = status_msg.x
        current_y = status_msg.y
        current_waypoint=0
        min_dis=float('inf')

        for i in range(len(global_path.poses)):
            dx=current_x - global_path.poses[i].pose.position.x
            dy=current_y - global_path.poses[i].pose.position.y
            dis=sqrt(dx*dx + dy*dy)
            if dis < min_dis :
                min_dis=dis
                current_waypoint=i

        if current_waypoint+10 > len(global_path.poses) :
            last_local_waypoint= len(global_path.poses) 
        else :
            last_local_waypoint=current_waypoint+10

        local_path.header.frame_id='map'
        for i in range(current_waypoint,last_local_waypoint):
            tmp_pose=PoseStamped()
            tmp_pose.pose.position.x=global_path.poses[i].pose.position.x
            tmp_pose.pose.position.y=global_path.poses[i].pose.position.y    
            tmp_pose.pose.position.z=global_path.poses[i].pose.position.z
            tmp_pose.pose.orientation.x=0
            tmp_pose.pose.orientation.y=0
            tmp_pose.pose.orientation.z=0
            tmp_pose.pose.orientation.w=1
            local_path.poses.append(tmp_pose)

        return local_path, current_waypoint



