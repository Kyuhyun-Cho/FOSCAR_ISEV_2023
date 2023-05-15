#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi


class pathReader :
    def __init__(self,pkg_name):
        rospack = rospkg.RosPack()
        self.file_path = rospack.get_path(pkg_name)

    def read_txt(self,file_name):
        full_file_name = self.file_path + "/path/" + file_name
        openFile = open(full_file_name, 'r')
        out_path=Path()
        
        out_path.header.frame_id='map'
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
            out_path.poses.append(read_pose)
        
        openFile.close()
        return out_path
      

# global_path에서  local_path 따로 구분하기
def findLocalPath(ref_path, odom_msg, start_waypoint):
    out_path=  Path()
    current_x = odom_msg.pose.pose.position.x
    current_y = odom_msg.pose.pose.position.y
    current_waypoint = 0
    min_dis = float('inf')

    # is_end_point = False

    end_waypoint = start_waypoint + 300
    if end_waypoint >= len(ref_path.poses):
        end_waypoint = len(ref_path.poses)

    for i in range(start_waypoint, end_waypoint) :
        dx = current_x - ref_path.poses[i].pose.position.x
        dy = current_y - ref_path.poses[i].pose.position.y
        dis = sqrt(dx*dx + dy*dy)

        # WeBot 기준 가장 가까운 waypoint 식별하기
        if dis < min_dis :
            min_dis = dis
            current_waypoint = i
  
    # print(current_waypoint, len(ref_path.poses))
    # 가장 가까운 waypoint에서 +50번째에 있는 waypoint까지를 하나의 local_path로 간주하기
    if current_waypoint + 100 > len(ref_path.poses) : # 만약 가장 가까운 waypoint에서 +100번째에 있는 waypoint를 찾고자 했으나 전체 global_path 인덱스를 초과한다면 도착점에 가까워진 것이므로 global_path의 마지막 waypoint를 local_path의 종점으로 활용
        last_local_waypoint = len(ref_path.poses)
    else :
        last_local_waypoint = current_waypoint + 100

    # if current_waypoint + 10 > len(ref_path.poses) :
    #     is_end_point = True


    out_path.header.frame_id = 'map'
    for i in range(current_waypoint,last_local_waypoint) :
        tmp_pose = PoseStamped()
        tmp_pose.pose.position.x = ref_path.poses[i].pose.position.x
        tmp_pose.pose.position.y = ref_path.poses[i].pose.position.y
        tmp_pose.pose.position.z = ref_path.poses[i].pose.position.z
        tmp_pose.pose.orientation.x = 0
        tmp_pose.pose.orientation.y = 0
        tmp_pose.pose.orientation.z = 0
        tmp_pose.pose.orientation.w = 1
        out_path.poses.append(tmp_pose)

    return out_path, current_waypoint 


class velocityPlanning :
    def __init__(self,car_max_speed,road_friction):
        self.car_max_speed = car_max_speed
        self.road_friction = road_friction
 
    def curveBasedVelocity(self,global_path,point_num):
        out_vel_plan = []
        for i in range(0,point_num):
            out_vel_plan.append(self.car_max_speed)

        for i in range(point_num,len(global_path.poses)-point_num):
            x_list = []
            y_list = []
            for box in  range(-point_num,point_num):
                x = global_path.poses[i+box].pose.position.x
                y = global_path.poses[i+box].pose.position.y
                x_list.append([-2*x,-2*y,1])
                y_list.append(-(x*x)-(y*y))
            
            x_matrix = np.array(x_list)
            y_matrix = np.array(y_list)
            x_trans = x_matrix.T
            
            a_matrix=np.linalg.inv(x_trans.dot(x_matrix)).dot(x_trans).dot(y_matrix)
            a=a_matrix[0]
            b=a_matrix[1]
            c=a_matrix[2]
            r=sqrt(a*a+b*b-c)
            v_max=sqrt(r*9.8*self.road_friction)  #0.7
            if v_max>self.car_max_speed :
                v_max=self.car_max_speed
            out_vel_plan.append(v_max)

        for i in range(len(global_path.poses)-point_num,len(global_path.poses)):
            out_vel_plan.append(self.car_max_speed)
        
        return out_vel_plan


class purePursuit :
    def __init__(self):
        self.forward_point=Point()
        self.current_postion=Point()
        self.is_look_forward_point=False
        self.lfd = 1
        self.min_lfd = 13
        self.max_lfd = 20
        self.vehicle_length = 4.635
        self.steering = 0
        
    def getPath(self,msg):
        self.path = msg  #nav_msgs/Path 
    
    def getEgoStatus(self, odom_msg, vehicle_vel):

        quaternion = (odom_msg.pose.pose.orientation.x,odom_msg.pose.pose.orientation.y,odom_msg.pose.pose.orientation.z,odom_msg.pose.pose.orientation.w)
        _, _, vehicle_yaw = euler_from_quaternion(quaternion)

        self.vehicle_yaw = vehicle_yaw
        self.current_vel = vehicle_vel

        # self.current_vel = 5
        # self.vehicle_yaw = 0.0   # rad
        self.current_postion.x = odom_msg.pose.pose.position.x
        self.current_postion.y = odom_msg.pose.pose.position.y
        self.current_postion.z = 0.0

    def steering_angle(self, lfd):
        vehicle_position = self.current_postion
        rotated_point = Point()
        self.is_look_forward_point = False

        self.lfd = 1
        # self.min_lfd = 13
        # self.max_lfd = 20

        for i in self.path.poses :
            path_point = i.pose.position
            
            dx = path_point.x - vehicle_position.x
            dy = path_point.y - vehicle_position.y
            
            rotated_point.x = cos(self.vehicle_yaw)*dx + sin(self.vehicle_yaw)*dy
            rotated_point.y = sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy
            
            
            if rotated_point.x > 0 :
                dis = sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))


                if dis >= self.lfd :
                    # self.lfd = self.current_vel 
                    self.lfd = lfd
                    # if self.lfd < self.min_lfd : 
                    #     self.lfd = self.min_lfd/2 -0.5
                    # elif self.lfd > self.max_lfd :
                    #     self.lfd = self.max_lfd

                    self.forward_point = path_point
                    self.is_look_forward_point=True
                    # print(dis, self.lfd)
                    break
             

        theta = atan2(rotated_point.y,rotated_point.x)
        # print('theta', theta)
        
    
        # print("LFD: ", self.lfd)
        self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi*(-1)
        # print('steering', self.steering)
        return self.steering, path_point.x, path_point.y
    
    def rear_steering_angle(self):
        vehicle_position = self.current_postion
        rotated_point = Point()
        self.is_look_forward_point = False
        

        self.lfd = 1
        self.min_lfd = 1
        self.max_lfd = 2

        for i in self.path.poses :
            path_point = i.pose.position
            # print(vehicle_position.x, vehicle_position.y)
            dx = path_point.x - vehicle_position.x
            dy = path_point.y - vehicle_position.y
            # print(path_point.x, path_point.y)
            # print(dx, dy)
            rotated_point.x = cos(self.vehicle_yaw)*dx + sin(self.vehicle_yaw)*dy
            rotated_point.y = sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy
            
            # print(rotated_point.x, rotated_point.y)

            if rotated_point.x > 0 :
                dis = sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))
                
                # print(dis, self.lfd)
                if dis >= self.lfd :
                    # self.lfd = self.current_vel / 1.8
                    self.lfd = self.current_vel / 1.8
                    if self.lfd < self.min_lfd : 
                        self.lfd=self.min_lfd
                    elif self.lfd > self.max_lfd :
                        self.lfd=self.max_lfd
                    self.forward_point=path_point
                    self.is_look_forward_point=True
                    break

        theta = atan2(rotated_point.y,rotated_point.x)
        # print(theta)
        # print("TARGET_X: ", rotated_point.x, "TARGET_Y: ", rotated_point.y)
        if self.is_look_forward_point :
            self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi*(-1)#deg
            # print(self.steering)
            return self.steering, path_point.x, path_point.y
        else : 
            self.steering = atan2((2*self.vehicle_length*sin(theta)),self.lfd)*180/pi*(-1)
            # print(self.steering)
            return self.steering, path_point.x, path_point.y


